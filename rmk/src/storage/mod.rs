use core::fmt::Debug;
use core::future::Future;

use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::Duration;
use embedded_storage::nor_flash::NorFlash;
use embedded_storage_async::nor_flash::NorFlash as AsyncNorFlash;
use rmk_types::connection::ConnectionType;
use rmk_types::morse::MorseProfile;
use sequential_storage::Error as SSError;
use sequential_storage::cache::Cache;
use sequential_storage::cache::key_pointers::ArrayKeyPointers;
use sequential_storage::cache::page_pointers::ArrayPagePointers;
use sequential_storage::cache::page_states::CalculatedPageStates;
use sequential_storage::map::{Key as MapKey, MapConfig, MapStorage, PostcardValue, SerializationError};
#[cfg(feature = "host")]
use {
    crate::{MACRO_SPACE_SIZE, keyboard::combo::ComboConfig},
    rmk_types::action::{EncoderAction, KeyAction},
    rmk_types::fork::Fork,
    rmk_types::morse::Morse,
};

#[cfg(feature = "_ble")]
use crate::ble::profile::ProfileInfo;
use crate::boot::reboot_keyboard;
use crate::config;
use crate::config::StorageConfig;
#[cfg(all(feature = "_ble", feature = "split"))]
use crate::split::ble::PeerAddress;

/// An operation request to the `Storage` task.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug)]
pub(crate) enum FlashOperationMessage {
    /// `Some(ticket)` is answered once the write has landed; `None` is fire-and-forget
    /// and the runner only logs a failure.
    Store(StorageItem, Option<u8>),
    /// Answered once on `REPLY` after every earlier message (FIFO).
    Read(StorageKey, u8),
    /// `erase_all` + reboot; never answered.
    Reset,
}

pub(crate) type Reply = Result<Option<StorageData>, ()>;

// Firmware code goes through `store`/`store_unchecked`/`read`/`reset`; `test_support` stands in for the task.
pub(crate) static FLASH_CHANNEL: Channel<crate::RawMutex, FlashOperationMessage, { crate::FLASH_CHANNEL_SIZE }> =
    Channel::new();
/// One ticketed request in flight: the lock is the turn, its value is the ticket counter.
static TURN: Mutex<crate::RawMutex, u8> = Mutex::new(0);
/// `(ticket, reply)`.
pub(crate) static REPLY: Signal<crate::RawMutex, (u8, Reply)> = Signal::new();

async fn request(build: impl FnOnce(u8) -> FlashOperationMessage) -> Reply {
    let mut turn = TURN.lock().await;
    *turn = turn.wrapping_add(1);
    let ticket = *turn;
    FLASH_CHANNEL.send(build(ticket)).await;
    // A predecessor cancelled after `send` leaves its reply in the slot first: skip it by ticket.
    loop {
        let (t, reply) = REPLY.wait().await;
        if t == ticket {
            return reply;
        }
    }
}

/// Write `item` and report whether it landed. Returns once the write is on flash.
pub(crate) async fn store(item: StorageItem) -> Result<(), ()> {
    request(|t| FlashOperationMessage::Store(item, Some(t)))
        .await
        .map(|_| ())
}

/// Write `item` without waiting for the outcome; a failure only reaches the log.
/// Not `async fn`: that would keep `item` alive beside the message inside the future.
pub(crate) fn store_unchecked(item: StorageItem) -> impl Future<Output = ()> {
    FLASH_CHANNEL.send(FlashOperationMessage::Store(item, None))
}

pub(crate) async fn read(key: StorageKey) -> Reply {
    request(|t| FlashOperationMessage::Read(key, t)).await
}

/// Erase everything and reboot.
pub(crate) async fn reset() {
    FLASH_CHANNEL.send(FlashOperationMessage::Reset).await
}

/// The most one user slot holds. Changing it is a format change, so the next
/// firmware reinitializes the storage on its own (see `Storage::firmware`).
pub const USER_DATA_MAX_SIZE: usize = 16;

/// Bytes a board persists for itself. RMK never looks inside one.
pub type UserData = heapless::Vec<u8, USER_DATA_MAX_SIZE>;

/// Persist `bytes` in board-defined slot `slot`, replacing whatever was there.
///
/// For state a board owns and RMK has no concept of — a trackball's learned
/// orientation, a mode the board invented. The write is unchecked, so a flash
/// failure reaches the log and not the caller.
///
/// `Err` when `bytes` is longer than [`USER_DATA_MAX_SIZE`].
pub async fn store_user_data(slot: u8, bytes: &[u8]) -> Result<(), heapless::CapacityError> {
    let data = UserData::from_slice(bytes)?;
    store_unchecked(StorageItem::UserData { slot, data }).await;
    Ok(())
}

/// Read back slot `slot`, `None` if nothing was ever stored there.
///
/// The storage task answers this, so it only works once `Storage` is running.
pub async fn read_user_data(slot: u8) -> Option<UserData> {
    match read(StorageKey::UserData(slot)).await {
        Ok(Some(StorageData::UserData(data))) => Some(data),
        _ => None,
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum StorageKey {
    StorageConfig,
    DefaultLayer,
    LayoutOption,
    BehaviorConfig,
    ConnectionType,
    #[cfg(feature = "host")]
    MacroData,
    #[cfg(feature = "host")]
    Keymap {
        layer: u8,
        row: u8,
        col: u8,
    },
    #[cfg(feature = "host")]
    Encoder {
        layer: u8,
        idx: u8,
    },
    #[cfg(feature = "host")]
    Combo(u8),
    #[cfg(feature = "host")]
    Fork(u8),
    #[cfg(feature = "host")]
    Morse(u8),
    #[cfg(all(feature = "_ble", feature = "split"))]
    PeerAddress(u8),
    #[cfg(feature = "_ble")]
    ActiveBleProfile,
    #[cfg(feature = "_ble")]
    BondInfo(u8),
    /// A slot the board defines, see [`store_user_data`].
    UserData(u8),
}

/// What a writer stores: the key and its value in one piece, so they cannot be mismatched.
/// `split` is the only place that maps it onto the on-flash `StorageKey`/`StorageData` pair.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug)]
pub(crate) enum StorageItem {
    StorageConfig {
        schema: u32,
        layout: u32,
    },
    DefaultLayer(u8),
    LayoutOption(u32),
    BehaviorConfig(BehaviorConfig),
    ConnectionType(ConnectionType),
    #[cfg(feature = "host")]
    MacroData([u8; MACRO_SPACE_SIZE]),
    #[cfg(feature = "host")]
    Keymap {
        layer: u8,
        row: u8,
        col: u8,
        action: KeyAction,
    },
    #[cfg(feature = "host")]
    Encoder {
        layer: u8,
        idx: u8,
        action: EncoderAction,
    },
    #[cfg(feature = "host")]
    Combo {
        idx: u8,
        config: ComboConfig,
    },
    #[cfg(feature = "host")]
    Fork {
        idx: u8,
        fork: Fork,
    },
    #[cfg(feature = "host")]
    Morse {
        idx: u8,
        morse: Morse,
    },
    #[cfg(all(feature = "_ble", feature = "split"))]
    PeerAddress(PeerAddress),
    #[cfg(feature = "_ble")]
    BondInfo(ProfileInfo),
    #[cfg(feature = "_ble")]
    ActiveBleProfile(u8),
    UserData {
        slot: u8,
        data: UserData,
    },
}

impl StorageItem {
    fn split(self) -> (StorageKey, StorageData) {
        match self {
            Self::StorageConfig { schema, layout } => {
                (StorageKey::StorageConfig, StorageData::StorageConfig { schema, layout })
            }
            Self::DefaultLayer(v) => (StorageKey::DefaultLayer, StorageData::DefaultLayer(v)),
            Self::LayoutOption(v) => (StorageKey::LayoutOption, StorageData::LayoutOption(v)),
            Self::BehaviorConfig(v) => (StorageKey::BehaviorConfig, StorageData::BehaviorConfig(v)),
            Self::ConnectionType(v) => (StorageKey::ConnectionType, StorageData::ConnectionType(v)),
            #[cfg(feature = "host")]
            Self::MacroData(v) => (StorageKey::MacroData, StorageData::MacroData(v)),
            #[cfg(feature = "host")]
            Self::Keymap {
                layer,
                row,
                col,
                action,
            } => (StorageKey::Keymap { layer, row, col }, StorageData::KeyAction(action)),
            #[cfg(feature = "host")]
            Self::Encoder { layer, idx, action } => {
                (StorageKey::Encoder { layer, idx }, StorageData::EncoderAction(action))
            }
            #[cfg(feature = "host")]
            Self::Combo { idx, config } => (StorageKey::Combo(idx), StorageData::Combo(config)),
            #[cfg(feature = "host")]
            Self::Fork { idx, fork } => (StorageKey::Fork(idx), StorageData::Fork(fork)),
            #[cfg(feature = "host")]
            Self::Morse { idx, morse } => (StorageKey::Morse(idx), StorageData::Morse(morse)),
            #[cfg(all(feature = "_ble", feature = "split"))]
            Self::PeerAddress(v) => (StorageKey::PeerAddress(v.peer_id), StorageData::PeerAddress(v)),
            #[cfg(feature = "_ble")]
            Self::BondInfo(v) => (StorageKey::BondInfo(v.slot_num), StorageData::BondInfo(v)),
            #[cfg(feature = "_ble")]
            Self::ActiveBleProfile(v) => (StorageKey::ActiveBleProfile, StorageData::ActiveBleProfile(v)),
            Self::UserData { slot, data } => (StorageKey::UserData(slot), StorageData::UserData(data)),
        }
    }
}

impl MapKey for StorageKey {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        postcard::to_slice(self, buffer)
            .map(|used| used.len())
            .map_err(Into::into)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        let (key, rest): (Self, &[u8]) = postcard::take_from_bytes(buffer).map_err(SerializationError::from)?;
        Ok((key, buffer.len() - rest.len()))
    }

    fn get_len(buffer: &[u8]) -> Result<usize, SerializationError> {
        Self::deserialize_from(buffer).map(|(_, len)| len)
    }
}

#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub(crate) enum StorageData {
    /// What the firmware that wrote this storage hashed, see `Storage::schema`.
    StorageConfig {
        schema: u32,
        layout: u32,
    },
    DefaultLayer(u8),
    LayoutOption(u32),
    BehaviorConfig(BehaviorConfig),
    ConnectionType(ConnectionType),
    #[cfg(feature = "host")]
    MacroData(#[serde(with = "crate::host::storage::macro_bytes_serde")] [u8; MACRO_SPACE_SIZE]),
    #[cfg(feature = "host")]
    KeyAction(KeyAction),
    #[cfg(feature = "host")]
    EncoderAction(EncoderAction),
    #[cfg(feature = "host")]
    Combo(ComboConfig),
    #[cfg(feature = "host")]
    Fork(Fork),
    #[cfg(feature = "host")]
    Morse(Morse),
    #[cfg(all(feature = "_ble", feature = "split"))]
    PeerAddress(PeerAddress),
    #[cfg(feature = "_ble")]
    BondInfo(ProfileInfo),
    #[cfg(feature = "_ble")]
    ActiveBleProfile(u8),
    UserData(UserData),
}

impl<'a> PostcardValue<'a> for StorageData {}

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct BehaviorConfig {
    // The prior-idle-time in ms used for in flow tap
    pub(crate) prior_idle_time: u16,
    // Default morse profile containing mode, timeouts, and unilateral_tap settings
    pub(crate) morse_default_profile: MorseProfile,

    // Timeout time for combos
    pub(crate) combo_timeout: u16,
    // Timeout time for one-shot keys
    pub(crate) one_shot_timeout: u16,
    // Interval for tap actions
    pub(crate) tap_interval: u16,
    // Interval for tapping capslock.
    // macOS has special processing of capslock, when tapping capslock, the tap interval should be another value
    pub(crate) tap_capslock_interval: u16,
}

impl From<&config::BehaviorConfig> for BehaviorConfig {
    fn from(behavior: &config::BehaviorConfig) -> Self {
        // Note: default_layer persists under its own key (restored in read_keymap), not this struct.
        Self {
            prior_idle_time: behavior.morse.prior_idle_time.as_millis() as u16,
            morse_default_profile: behavior.morse.default_profile,
            combo_timeout: behavior.combo.timeout.as_millis() as u16,
            one_shot_timeout: behavior.one_shot.timeout.as_millis() as u16,
            tap_interval: behavior.tap.tap_interval,
            tap_capslock_interval: behavior.tap.tap_capslock_interval,
        }
    }
}

pub fn async_flash_wrapper<F: NorFlash>(flash: F) -> BlockingAsync<F> {
    embassy_embedded_hal::adapter::BlockingAsync::new(flash)
}

/// Storage for the firmwares that hold no keymap of their own — a split
/// peripheral and a dongle. Both still persist their BLE bonds, which the
/// profile manager loads over `FLASH_CHANNEL`.
#[cfg(any(feature = "split", feature = "dongle"))]
pub async fn new_storage_without_keymap<F: AsyncNorFlash>(
    flash: F,
    storage_config: StorageConfig,
) -> Storage<F, 0, 0, 0, 0> {
    Storage::<F, 0, 0, 0, 0>::new(
        flash,
        #[cfg(feature = "host")]
        &[],
        #[cfg(feature = "host")]
        &None,
        &storage_config,
        &config::BehaviorConfig::default(),
    )
    .await
}

/// Page caches make every store O(1) in page lookups; the key cache serves runtime
/// reads and the per-item lookups of a page migration. 32 page slots cover every
/// chip default (pages beyond run uncached); 32 key slots (8 B each) cover the
/// runtime readers, not a whole keymap.
type StorageCache = Cache<CalculatedPageStates, ArrayPagePointers<32>, ArrayKeyPointers<StorageKey, 32>, StorageKey>;

pub struct Storage<
    F: AsyncNorFlash,
    const ROW: usize,
    const COL: usize,
    const NUM_LAYER: usize,
    const NUM_ENCODER: usize = 0,
> {
    pub(crate) flash: MapStorage<StorageKey, F, StorageCache>,
    pub(crate) buffer: [u8; get_buffer_size()],
    /// FNV-1a over what decides whether stored bytes can be read back at all: the rmk
    /// version, its commit, and the feature set that gates the two enums' variants. A
    /// mismatch means an item could decode as the wrong variant, so the storage is erased.
    pub(crate) schema: u32,
    /// FNV-1a over the compiled-in layout: geometry, keymap, encoder map, behavior
    /// defaults, combos, forks, morses and macros. A mismatch rewrites only what the
    /// layout owns, so a keymap edited in source shows up without dropping pairings.
    pub(crate) layout: u32,
}

impl<F: AsyncNorFlash, const ROW: usize, const COL: usize, const NUM_LAYER: usize, const NUM_ENCODER: usize>
    Storage<F, ROW, COL, NUM_LAYER, NUM_ENCODER>
{
    pub(crate) async fn fetch(&mut self, key: StorageKey) -> Reply {
        self.flash
            .fetch_item(&mut self.buffer, &key)
            .await
            .map_err(|e| print_storage_error::<F>(e))
    }

    // Like `store`: split first so the future holds the pair, not the pair and `item`.
    fn put(&mut self, item: StorageItem) -> impl Future<Output = Result<(), SSError<F::Error>>> {
        let (key, data) = item.split();
        async move {
            self.flash
                .store_item(&mut self.buffer, &key, &data)
                .await
                .inspect_err(|_| error!("Failed to store {:?}", key))
        }
    }

    pub async fn new(
        flash: F,
        #[cfg(feature = "host")] keymap: &[[[KeyAction; COL]; ROW]; NUM_LAYER],
        #[cfg(feature = "host")] encoder_map: &Option<&mut [[EncoderAction; NUM_ENCODER]; NUM_LAYER]>,
        storage_config: &StorageConfig,
        behavior_config: &config::BehaviorConfig,
    ) -> Self {
        assert!(
            storage_config.num_sectors >= 2,
            "Number of used sector for storage must larger than 1"
        );

        // `start_addr == 0` means the last `num_sectors` sectors, except on nRF BLE builds
        // without DFU, which keep the historical 0x6_0000; with DFU the partition is placed by rmk-boot.
        #[cfg(all(feature = "_nrf_ble", not(feature = "_dfu")))]
        let start_addr = if storage_config.start_addr == 0 {
            0x0006_0000
        } else {
            storage_config.start_addr
        };
        #[cfg(not(all(feature = "_nrf_ble", not(feature = "_dfu"))))]
        let start_addr = storage_config.start_addr;

        let storage_range = if start_addr == 0 {
            (flash.capacity() - storage_config.num_sectors as usize * F::ERASE_SIZE) as u32..flash.capacity() as u32
        } else {
            assert!(
                start_addr.is_multiple_of(F::ERASE_SIZE),
                "Storage's start addr MUST BE a multiplier of sector size"
            );
            start_addr as u32..(start_addr + storage_config.num_sectors as usize * F::ERASE_SIZE) as u32
        };
        info!(
            "Flash capacity {} KB, RMK use {} KB({} sectors) starting from 0x{:X} as storage",
            flash.capacity() / 1024,
            (F::ERASE_SIZE * storage_config.num_sectors as usize) / 1024,
            storage_config.num_sectors,
            storage_range.start,
        );

        let cache = || {
            Cache::new(
                CalculatedPageStates::new(storage_config.num_sectors as usize),
                ArrayPagePointers::new(),
                ArrayKeyPointers::new(),
            )
        };
        let mut buffer = [0; get_buffer_size()];
        let mix = |h: &mut u32, bytes: &[u8]| {
            for byte in bytes {
                *h = (*h ^ *byte as u32).wrapping_mul(0x0100_0193);
            }
        };
        let mut schema = 0x811c_9dc5u32;
        mix(&mut schema, env!("CARGO_PKG_VERSION").as_bytes());
        mix(&mut schema, env!("RMK_COMMIT").as_bytes());
        // Features gate variants of the two enums, shifting their postcard tags.
        mix(&mut schema, env!("RMK_FEATURES").as_bytes());
        // `keyboard.toml` sizes decide how a stored value is framed.
        #[cfg(feature = "host")]
        for size in [MACRO_SPACE_SIZE, crate::COMBO_SIZE, crate::MORSE_SIZE] {
            mix(&mut schema, &(size as u32).to_le_bytes());
        }

        let mut layout = 0x811c_9dc5u32;
        mix(&mut layout, &[ROW as u8, COL as u8, NUM_LAYER as u8, NUM_ENCODER as u8]);
        mix(
            &mut layout,
            postcard::to_slice(&BehaviorConfig::from(behavior_config), &mut buffer).unwrap(),
        );
        #[cfg(feature = "host")]
        {
            for action in keymap.as_flattened().as_flattened() {
                mix(&mut layout, postcard::to_slice(action, &mut buffer).unwrap());
            }
            if let Some(encoder_map) = encoder_map {
                for action in encoder_map.as_flattened() {
                    mix(&mut layout, postcard::to_slice(action, &mut buffer).unwrap());
                }
            }
            // Hashed exactly as `write_layout` writes them, so the two cannot drift.
            for combo in behavior_config.combo.combos.iter() {
                let config = combo.as_ref().map_or_else(ComboConfig::empty, |c| c.config.clone());
                mix(&mut layout, postcard::to_slice(&config, &mut buffer).unwrap());
            }
            for fork in behavior_config.fork.forks.iter() {
                mix(&mut layout, postcard::to_slice(fork, &mut buffer).unwrap());
            }
            for morse in behavior_config.morse.morses.iter() {
                mix(&mut layout, postcard::to_slice(morse, &mut buffer).unwrap());
            }
            mix(&mut layout, &behavior_config.keyboard_macros.macro_sequences);
        }

        let mut storage = Self {
            flash: MapStorage::new(flash, MapConfig::new(storage_range.clone()), cache()),
            buffer,
            schema,
            layout,
        };

        let stored = storage.fetch(StorageKey::StorageConfig).await;
        let wipe = storage_config.clear_storage
            || !matches!(stored, Ok(Some(StorageData::StorageConfig { schema, .. })) if schema == storage.schema);
        let relayout = !wipe
            && (storage_config.clear_layout
                || !matches!(stored, Ok(Some(StorageData::StorageConfig { layout, .. })) if layout == storage.layout));

        if wipe {
            debug!("Clearing storage!");
            // The probe taught the cache the old page layout and an erase never touches a
            // cache, so the map is rebuilt with a fresh one over the erased range.
            // Nothing is written back: with the range erased every read misses and the
            // compiled-in defaults already in RAM stand.
            let (mut raw, _) = storage.flash.destroy();
            let _ = raw.erase(storage_range.start, storage_range.end).await;
            storage.flash = MapStorage::new(raw, MapConfig::new(storage_range), cache());
        } else if relayout {
            // Only the layout changed, so its items are overwritten where they lie and
            // everything else — pairings, connection type, user slots — is left alone.
            debug!("Layout changed, rewriting the items it owns.");
            #[cfg(feature = "host")]
            storage.write_layout(keymap, encoder_map, behavior_config).await;
        }
        if wipe || relayout {
            // Written last: a boot after a partial rewrite finds no match and starts over.
            let _ = storage
                .put(StorageItem::StorageConfig {
                    schema: storage.schema,
                    layout: storage.layout,
                })
                .await;
        }

        storage
    }

    pub(crate) async fn read_behavior_config(
        &mut self,
        behavior_config: &mut config::BehaviorConfig,
    ) -> Result<(), ()> {
        if let Some(StorageData::BehaviorConfig(c)) = self.fetch(StorageKey::BehaviorConfig).await? {
            behavior_config.morse.prior_idle_time = Duration::from_millis(c.prior_idle_time as u64);
            behavior_config.morse.default_profile = c.morse_default_profile;

            behavior_config.combo.timeout = Duration::from_millis(c.combo_timeout as u64);
            behavior_config.one_shot.timeout = Duration::from_millis(c.one_shot_timeout as u64);
            behavior_config.tap.tap_interval = c.tap_interval;
            behavior_config.tap.tap_capslock_interval = c.tap_capslock_interval;
        }

        Ok(())
    }

    /// Overwrite every item the layout owns with the compiled-in defaults, so a value a
    /// host wrote earlier stops shadowing what was flashed. Only reached when nothing was
    /// erased; the caller records the new hashes afterwards.
    #[cfg(feature = "host")]
    async fn write_layout(
        &mut self,
        keymap: &[[[KeyAction; COL]; ROW]; NUM_LAYER],
        encoder_map: &Option<&mut [[EncoderAction; NUM_ENCODER]; NUM_LAYER]>,
        behavior: &config::BehaviorConfig,
    ) {
        let mut put = async |item| {
            if let Err(e) = self.put(item).await {
                print_storage_error::<F>(e);
            }
        };
        put(StorageItem::BehaviorConfig(behavior.into())).await;
        put(StorageItem::DefaultLayer(0)).await;
        put(StorageItem::LayoutOption(0)).await;
        put(StorageItem::MacroData(behavior.keyboard_macros.macro_sequences)).await;

        for (layer, layer_data) in keymap.iter().enumerate() {
            for (row, row_data) in layer_data.iter().enumerate() {
                for (col, action) in row_data.iter().enumerate() {
                    put(StorageItem::Keymap {
                        layer: layer as u8,
                        row: row as u8,
                        col: col as u8,
                        action: *action,
                    })
                    .await;
                }
            }
        }
        if let Some(encoder_map) = encoder_map {
            for (layer, layer_data) in encoder_map.iter().enumerate() {
                for (idx, action) in layer_data.iter().enumerate() {
                    put(StorageItem::Encoder {
                        layer: layer as u8,
                        idx: idx as u8,
                        action: *action,
                    })
                    .await;
                }
            }
        }
        // Mirrors the hash in `new`: an empty slot is written as an empty config.
        for (idx, combo) in behavior.combo.combos.iter().enumerate() {
            let config = combo.as_ref().map_or_else(ComboConfig::empty, |c| c.config.clone());
            put(StorageItem::Combo { idx: idx as u8, config }).await;
        }
        for (idx, fork) in behavior.fork.forks.iter().enumerate() {
            put(StorageItem::Fork {
                idx: idx as u8,
                fork: *fork,
            })
            .await;
        }
        for (idx, morse) in behavior.morse.morses.iter().enumerate() {
            put(StorageItem::Morse {
                idx: idx as u8,
                morse: morse.clone(),
            })
            .await;
        }
    }
}

impl<F: AsyncNorFlash, const ROW: usize, const COL: usize, const NUM_LAYER: usize, const NUM_ENCODER: usize>
    crate::core_traits::Runnable for Storage<F, ROW, COL, NUM_LAYER, NUM_ENCODER>
{
    async fn run(&mut self) -> ! {
        loop {
            let (ticket, result) = match FLASH_CHANNEL.receive().await {
                FlashOperationMessage::Store(item, ack) => {
                    let result = self.put(item).await.map(|_| None).map_err(print_storage_error::<F>);
                    match ack {
                        Some(ticket) => (ticket, result),
                        None => continue,
                    }
                }
                FlashOperationMessage::Read(key, ticket) => (ticket, self.fetch(key).await),
                FlashOperationMessage::Reset => {
                    let _ = self.flash.erase_all().await;
                    reboot_keyboard();
                    // Only `std` returns from reboot; the cache no longer describes the flash, so serve nothing.
                    core::future::pending().await
                }
            };
            REPLY.signal((ticket, result));
        }
    }
}

pub(crate) fn print_storage_error<F: AsyncNorFlash>(e: SSError<F::Error>) {
    match e {
        #[cfg(feature = "defmt")]
        SSError::Storage { value: e } => error!("Flash error: {:?}", defmt::Debug2Format(&e)),
        #[cfg(not(feature = "defmt"))]
        SSError::Storage { value: _e } => error!("Flash error"),
        SSError::FullStorage => error!("Storage is full"),
        SSError::Corrupted {} => error!("Storage is corrupted"),
        SSError::BufferTooBig => error!("Buffer too big"),
        SSError::BufferTooSmall(x) => error!("Buffer too small, needs {} bytes", x),
        SSError::SerializationError(e) => error!("Map value error: {}", e),
        SSError::ItemTooBig => error!("Item too big"),
        _ => error!("Unknown storage error"),
    }
}

const fn get_buffer_size() -> usize {
    #[cfg(feature = "host")]
    {
        // The largest item is the macro buffer plus its framing; `sequential-storage`
        // wants 32-byte alignment on some flashes, so always round up.
        let buffer_size = if crate::MACRO_SPACE_SIZE < 248 {
            256
        } else {
            crate::MACRO_SPACE_SIZE + 8
        };
        (buffer_size + 31) & !31
    }

    #[cfg(not(feature = "host"))]
    256
}

#[cfg(test)]
mod tests {
    use core::future::Future;
    use core::pin::pin;
    use core::sync::atomic::{AtomicBool, Ordering};
    use core::task::{Context, Poll, Waker};

    use embassy_futures::select::{Either, select};
    use sequential_storage::cache::Cache;
    use sequential_storage::map::{MapConfig, MapStorage};

    use super::*;
    use crate::config::{BehaviorConfig as RuntimeBehaviorConfig, StorageConfig as RuntimeStorageConfig};
    use crate::test_support::test_block_on as block_on;

    /// Makes every `TestFlash::write` fail while set.
    static FAIL_WRITES: AtomicBool = AtomicBool::new(false);

    #[derive(Debug, Clone, Copy)]
    struct TestFlashError;

    impl embedded_storage_async::nor_flash::NorFlashError for TestFlashError {
        fn kind(&self) -> embedded_storage_async::nor_flash::NorFlashErrorKind {
            embedded_storage_async::nor_flash::NorFlashErrorKind::Other
        }
    }

    struct TestFlash<const SIZE: usize, const ERASE_SIZE: usize, const WRITE_SIZE: usize> {
        bytes: [u8; SIZE],
    }

    impl<const SIZE: usize, const ERASE_SIZE: usize, const WRITE_SIZE: usize> TestFlash<SIZE, ERASE_SIZE, WRITE_SIZE> {
        fn new() -> Self {
            Self { bytes: [0xFF; SIZE] }
        }
    }

    impl<const SIZE: usize, const ERASE_SIZE: usize, const WRITE_SIZE: usize> embedded_storage::nor_flash::ErrorType
        for TestFlash<SIZE, ERASE_SIZE, WRITE_SIZE>
    {
        type Error = TestFlashError;
    }

    impl<const SIZE: usize, const ERASE_SIZE: usize, const WRITE_SIZE: usize> embedded_storage::nor_flash::ReadNorFlash
        for TestFlash<SIZE, ERASE_SIZE, WRITE_SIZE>
    {
        const READ_SIZE: usize = 1;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            let start = offset as usize;
            let end = start + bytes.len();
            bytes.copy_from_slice(&self.bytes[start..end]);
            Ok(())
        }

        fn capacity(&self) -> usize {
            SIZE
        }
    }

    impl<const SIZE: usize, const ERASE_SIZE: usize, const WRITE_SIZE: usize> embedded_storage::nor_flash::NorFlash
        for TestFlash<SIZE, ERASE_SIZE, WRITE_SIZE>
    {
        const WRITE_SIZE: usize = WRITE_SIZE;
        const ERASE_SIZE: usize = ERASE_SIZE;

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            self.bytes[from as usize..to as usize].fill(0xFF);
            Ok(())
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            if FAIL_WRITES.load(Ordering::Relaxed) {
                return Err(TestFlashError);
            }
            let start = offset as usize;
            let end = start + bytes.len();
            for (dst, src) in self.bytes[start..end].iter_mut().zip(bytes.iter()) {
                *dst &= *src;
            }
            Ok(())
        }
    }

    impl<const SIZE: usize, const ERASE_SIZE: usize, const WRITE_SIZE: usize>
        embedded_storage_async::nor_flash::ReadNorFlash for TestFlash<SIZE, ERASE_SIZE, WRITE_SIZE>
    {
        const READ_SIZE: usize = 1;

        async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            embedded_storage::nor_flash::ReadNorFlash::read(self, offset, bytes)
        }

        fn capacity(&self) -> usize {
            SIZE
        }
    }

    impl<const SIZE: usize, const ERASE_SIZE: usize, const WRITE_SIZE: usize>
        embedded_storage_async::nor_flash::NorFlash for TestFlash<SIZE, ERASE_SIZE, WRITE_SIZE>
    {
        const WRITE_SIZE: usize = WRITE_SIZE;
        const ERASE_SIZE: usize = ERASE_SIZE;

        async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            embedded_storage::nor_flash::NorFlash::erase(self, from, to)
        }

        async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            embedded_storage::nor_flash::NorFlash::write(self, offset, bytes)
        }
    }

    // The primitive tests below poll by hand: `test_block_on` re-polls with a noop
    // waker every step, so it would also pass a primitive that loses wake-ups.
    fn take_ticket() -> u8 {
        match FLASH_CHANNEL.try_receive() {
            Ok(FlashOperationMessage::Read(_, t)) | Ok(FlashOperationMessage::Store(_, Some(t))) => t,
            other => panic!("expected a ticketed request, got {other:?}"),
        }
    }

    #[test]
    fn request_skips_a_cancelled_predecessors_reply() {
        let mut cx = Context::from_waker(Waker::noop());
        crate::test_support::clear_flash_channel();

        let stale = {
            let mut first = pin!(read(StorageKey::StorageConfig));
            assert!(first.as_mut().poll(&mut cx).is_pending());
            take_ticket()
            // Dropped after `send`: the turn is released, the reply still arrives.
        };

        let mut second = pin!(read(StorageKey::StorageConfig));
        assert!(second.as_mut().poll(&mut cx).is_pending());
        let live = take_ticket();
        assert_ne!(stale, live);

        REPLY.signal((stale, Ok(None)));
        assert!(second.as_mut().poll(&mut cx).is_pending());
        REPLY.signal((live, Ok(None)));
        assert!(matches!(second.as_mut().poll(&mut cx), Poll::Ready(Ok(None))));
    }

    #[test]
    fn second_requester_waits_for_the_turn() {
        let mut cx = Context::from_waker(Waker::noop());
        crate::test_support::clear_flash_channel();

        let mut first = pin!(read(StorageKey::StorageConfig));
        let mut second = pin!(store(StorageItem::LayoutOption(1)));
        assert!(first.as_mut().poll(&mut cx).is_pending());
        assert!(second.as_mut().poll(&mut cx).is_pending());
        // Only the turn holder's message is in flight.
        let ticket = take_ticket();
        assert!(FLASH_CHANNEL.try_receive().is_err());

        REPLY.signal((ticket, Ok(None)));
        assert!(matches!(first.as_mut().poll(&mut cx), Poll::Ready(Ok(None))));
        assert!(second.as_mut().poll(&mut cx).is_pending());
        assert!(matches!(
            FLASH_CHANNEL.try_receive(),
            Ok(FlashOperationMessage::Store(_, Some(_)))
        ));
    }

    #[cfg(all(feature = "_ble", feature = "split"))]
    #[test]
    fn peer_address_write_waits_for_its_own_reply() {
        let mut cx = Context::from_waker(Waker::noop());
        crate::test_support::clear_flash_channel();
        FLASH_CHANNEL
            .try_send(FlashOperationMessage::Store(StorageItem::LayoutOption(42), None))
            .unwrap();

        let mut write = pin!(store(StorageItem::PeerAddress(PeerAddress::new(0, true, [1; 6]))));
        assert!(write.as_mut().poll(&mut cx).is_pending());

        // The storage task sees the older fire-and-forget write first, then this one.
        assert!(matches!(
            FLASH_CHANNEL.try_receive(),
            Ok(FlashOperationMessage::Store(StorageItem::LayoutOption(42), None))
        ));
        let ticket = match FLASH_CHANNEL.try_receive() {
            Ok(FlashOperationMessage::Store(StorageItem::PeerAddress(_), Some(t))) => t,
            other => panic!("expected the peer address write, got {other:?}"),
        };
        assert!(write.as_mut().poll(&mut cx).is_pending());
        REPLY.signal((ticket, Ok(None)));
        assert!(matches!(write.as_mut().poll(&mut cx), Poll::Ready(Ok(()))));
    }

    type Flash = TestFlash<16_384, 4_096, 1>;

    // Boxed: `TestFlash` is 16 KB by value and the `new` future copies it several times.
    async fn new_storage(flash: Flash) -> Storage<Flash, 1, 1, 1, 0> {
        #[cfg(feature = "host")]
        return new_storage_with_keymap(flash, [[[KeyAction::No; 1]; 1]; 1]).await;
        #[cfg(not(feature = "host"))]
        Box::pin(Storage::<Flash, 1, 1, 1, 0>::new(
            flash,
            &RuntimeStorageConfig::default(),
            &RuntimeBehaviorConfig::default(),
        ))
        .await
    }

    #[cfg(feature = "host")]
    async fn new_storage_with_keymap(flash: Flash, keymap: [[[KeyAction; 1]; 1]; 1]) -> Storage<Flash, 1, 1, 1, 0> {
        let encoder_map: Option<&mut [[EncoderAction; 0]; 1]> = None;
        Box::pin(Storage::<Flash, 1, 1, 1, 0>::new(
            flash,
            &keymap,
            &encoder_map,
            &RuntimeStorageConfig::default(),
            &RuntimeBehaviorConfig::default(),
        ))
        .await
    }

    /// A config item written by some other firmware.
    const STALE_CONFIG: StorageData = StorageData::StorageConfig { schema: 0, layout: 0 };

    const STORAGE_RANGE: core::ops::Range<u32> = (16_384 - 2 * 4_096) as u32..16_384u32;

    /// A flash holding `items`, written by an uncached map so `Storage::new` boots over them.
    async fn seeded(items: &[(StorageKey, StorageData)]) -> Flash {
        let mut map =
            MapStorage::<StorageKey, _, _>::new(Flash::new(), MapConfig::new(STORAGE_RANGE), Cache::new_uncached());
        let mut buffer = [0u8; 256];
        for (key, data) in items {
            map.store_item(&mut buffer, key, data).await.unwrap();
        }
        map.destroy().0
    }

    /// Run `body` against a live storage task over a fresh flash.
    fn with_storage_task<T>(body: impl Future<Output = T>) -> T {
        use crate::core_traits::Runnable;

        crate::test_support::clear_flash_channel();
        FAIL_WRITES.store(false, Ordering::Relaxed);
        block_on(async {
            let mut storage = new_storage(Flash::new()).await;
            match select(storage.run(), body).await {
                Either::First(never) => never,
                Either::Second(out) => out,
            }
        })
    }

    #[test]
    fn read_sees_write_queued_before_it() {
        with_storage_task(async {
            store_unchecked(StorageItem::ConnectionType(ConnectionType::Usb)).await;
            assert!(matches!(
                read(StorageKey::ConnectionType).await,
                Ok(Some(StorageData::ConnectionType(ConnectionType::Usb)))
            ));
            store_unchecked(StorageItem::ConnectionType(ConnectionType::Ble)).await;
            assert!(matches!(
                read(StorageKey::ConnectionType).await,
                Ok(Some(StorageData::ConnectionType(ConnectionType::Ble)))
            ));
        });
    }

    #[test]
    fn user_data_round_trips_through_its_slot() {
        with_storage_task(async {
            assert_eq!(read_user_data(3).await, None, "an untouched slot reads back empty");

            store_user_data(3, &[0xAA, 0x55]).await.unwrap();
            assert_eq!(read_user_data(3).await.as_deref(), Some(&[0xAA, 0x55][..]));

            // Slots are independent, and a second store replaces the first.
            store_user_data(4, &[1]).await.unwrap();
            store_user_data(3, &[9, 8, 7]).await.unwrap();
            assert_eq!(read_user_data(3).await.as_deref(), Some(&[9, 8, 7][..]));
            assert_eq!(read_user_data(4).await.as_deref(), Some(&[1][..]));

            assert!(
                store_user_data(3, &[0; USER_DATA_MAX_SIZE + 1]).await.is_err(),
                "too long for a slot, and nothing is written"
            );
        });
    }

    #[test]
    fn store_reports_its_own_failure() {
        with_storage_task(async {
            FAIL_WRITES.store(true, Ordering::Relaxed);
            assert!(store(StorageItem::ConnectionType(ConnectionType::Usb)).await.is_err());
            // A failed write is not sticky: the next one answers for itself.
            FAIL_WRITES.store(false, Ordering::Relaxed);
            assert!(store(StorageItem::ConnectionType(ConnectionType::Usb)).await.is_ok());
            assert!(read(StorageKey::ConnectionType).await.is_ok());
        });
    }

    // Without the rebuild in `Storage::new`, the cache still describes the page
    // layout the probe saw before the erase and the first store lands in a page
    // whose marker is gone: an uncached map over the same bytes cannot see it.
    #[test]
    fn reinit_writes_survive_a_fresh_map() {
        block_on(async {
            let flash = seeded(&[(StorageKey::StorageConfig, STALE_CONFIG)]).await;
            let mut storage = new_storage(flash).await;
            storage
                .put(StorageItem::ConnectionType(ConnectionType::Ble))
                .await
                .unwrap();

            let (flash, _) = storage.flash.destroy();
            let mut fresh =
                MapStorage::<StorageKey, _, _>::new(flash, MapConfig::new(STORAGE_RANGE), Cache::new_uncached());
            let mut buffer = [0u8; 256];
            assert!(matches!(
                fresh
                    .fetch_item::<StorageData>(&mut buffer, &StorageKey::ConnectionType)
                    .await,
                Ok(Some(StorageData::ConnectionType(ConnectionType::Ble)))
            ));
        });
    }

    #[test]
    fn firmware_mismatch_reinitializes_storage() {
        block_on(async {
            let flash = seeded(&[
                (StorageKey::StorageConfig, STALE_CONFIG),
                (StorageKey::DefaultLayer, StorageData::DefaultLayer(7)),
                (StorageKey::LayoutOption, StorageData::LayoutOption(42)),
            ])
            .await;
            let mut storage = new_storage(flash).await;

            // The mismatch wiped the layout items; the config item was rewritten for this firmware.
            assert!(matches!(storage.fetch(StorageKey::DefaultLayer).await, Ok(None)));
            assert!(matches!(storage.fetch(StorageKey::LayoutOption).await, Ok(None)));
            assert!(matches!(
                storage.fetch(StorageKey::StorageConfig).await,
                Ok(Some(StorageData::StorageConfig { schema, .. })) if schema == storage.schema
            ));
        });
    }

    /// A schema mismatch is the only thing that erases. It has to take the pairings with
    /// it, because an item written under another schema can decode as the wrong variant.
    #[test]
    fn schema_mismatch_drops_even_the_pairings() {
        block_on(async {
            let flash = seeded(&[
                (StorageKey::StorageConfig, STALE_CONFIG),
                (
                    StorageKey::ConnectionType,
                    StorageData::ConnectionType(ConnectionType::Ble),
                ),
            ])
            .await;
            let mut storage = new_storage(flash).await;
            assert!(matches!(storage.fetch(StorageKey::ConnectionType).await, Ok(None)));
        });
    }

    /// The same firmware keeps a keymap edit across `Storage::new`; a firmware with
    /// another compiled-in keymap overwrites it, so the flashed keymap is never shadowed.
    /// Only the layout changed, so the connection type — and with it a real board's
    /// pairings — survives.
    #[cfg(feature = "host")]
    #[test]
    fn keymap_change_reinitializes_storage() {
        use rmk_types::action::Action;
        use rmk_types::keycode::{HidKeyCode, KeyCode};

        const KEY: StorageKey = StorageKey::Keymap {
            layer: 0,
            row: 0,
            col: 0,
        };
        let a = KeyAction::Single(Action::Key(KeyCode::Hid(HidKeyCode::A)));
        let b = KeyAction::Single(Action::Key(KeyCode::Hid(HidKeyCode::B)));

        block_on(async {
            let mut storage = new_storage(Flash::new()).await;
            storage
                .put(StorageItem::Keymap {
                    layer: 0,
                    row: 0,
                    col: 0,
                    action: a,
                })
                .await
                .unwrap();
            storage
                .put(StorageItem::ConnectionType(ConnectionType::Ble))
                .await
                .unwrap();

            let (flash, _) = storage.flash.destroy();
            let mut storage = new_storage(flash).await;
            assert!(matches!(
                storage.fetch(KEY).await,
                Ok(Some(StorageData::KeyAction(action))) if action == a
            ));

            let (flash, _) = storage.flash.destroy();
            let mut storage = new_storage_with_keymap(flash, [[[b]]]).await;
            assert!(matches!(
                storage.fetch(KEY).await,
                Ok(Some(StorageData::KeyAction(action))) if action == b
            ));
            assert!(matches!(
                storage.fetch(StorageKey::ConnectionType).await,
                Ok(Some(StorageData::ConnectionType(ConnectionType::Ble)))
            ));
        });
    }

    // Postcard tags are declaration positions: inserting or reordering a variant
    // shifts every later tag and misreads storage written by the same commit.
    // Both enums are append-only within a commit.
    #[test]
    fn storage_variant_order_is_pinned() {
        use sequential_storage::map::Value;

        let keys = [
            StorageKey::StorageConfig,
            StorageKey::DefaultLayer,
            StorageKey::LayoutOption,
            StorageKey::BehaviorConfig,
            StorageKey::ConnectionType,
            #[cfg(feature = "host")]
            StorageKey::MacroData,
            #[cfg(feature = "host")]
            StorageKey::Keymap {
                layer: 2,
                row: 3,
                col: 4,
            },
            #[cfg(feature = "host")]
            StorageKey::Encoder { layer: 1, idx: 5 },
            #[cfg(feature = "host")]
            StorageKey::Combo(6),
            #[cfg(feature = "host")]
            StorageKey::Fork(7),
            #[cfg(feature = "host")]
            StorageKey::Morse(8),
            #[cfg(all(feature = "_ble", feature = "split"))]
            StorageKey::PeerAddress(9),
            #[cfg(feature = "_ble")]
            StorageKey::ActiveBleProfile,
            #[cfg(feature = "_ble")]
            StorageKey::BondInfo(10),
        ];
        let mut buffer = [0u8; 64];
        for (tag, key) in keys.iter().enumerate() {
            let size = MapKey::serialize_into(key, &mut buffer).unwrap();
            assert_eq!(buffer[0], tag as u8, "{key:?}");
            assert_eq!(MapKey::deserialize_from(&buffer[..size]).unwrap(), (*key, size));
        }

        let data = [
            STALE_CONFIG,
            StorageData::DefaultLayer(0),
            StorageData::LayoutOption(0),
            StorageData::BehaviorConfig((&RuntimeBehaviorConfig::default()).into()),
            StorageData::ConnectionType(ConnectionType::Usb),
            #[cfg(feature = "host")]
            StorageData::MacroData([0; MACRO_SPACE_SIZE]),
            #[cfg(feature = "host")]
            StorageData::KeyAction(KeyAction::No),
            #[cfg(feature = "host")]
            StorageData::EncoderAction(EncoderAction::default()),
            #[cfg(feature = "host")]
            StorageData::Combo(ComboConfig::empty()),
            #[cfg(feature = "host")]
            StorageData::Fork(Fork::default()),
            #[cfg(feature = "host")]
            StorageData::Morse(Morse::default()),
            #[cfg(all(feature = "_ble", feature = "split"))]
            StorageData::PeerAddress(PeerAddress::new(0, false, [0; 6])),
            #[cfg(feature = "_ble")]
            StorageData::BondInfo(ProfileInfo::default()),
            #[cfg(feature = "_ble")]
            StorageData::ActiveBleProfile(0),
        ];
        let mut buffer = [0u8; get_buffer_size()];
        for (tag, item) in data.iter().enumerate() {
            Value::serialize_into(item, &mut buffer).unwrap();
            assert_eq!(buffer[0], tag as u8, "{item:?}");
        }
    }

    // A stored LayoutOption must reach the Vial GUI after a power cycle: `read_keymap`
    // restores it into `KeymapData` and `KeyMap::new` copies it into the state
    // `GetKeyboardValue` answers from. Drop either step and this reads 0.
    #[cfg(feature = "vial")]
    #[test]
    fn layout_option_restored_from_storage() {
        use crate::config::BehaviorConfig;
        use crate::keymap::{KeyMap, KeymapData};

        block_on(async {
            // A matching config item keeps the stored records across the boot.
            let stored = new_storage(Flash::new()).await;
            let (schema, layout) = (stored.schema, stored.layout);
            let flash = seeded(&[
                (StorageKey::StorageConfig, StorageData::StorageConfig { schema, layout }),
                (StorageKey::LayoutOption, StorageData::LayoutOption(42)),
            ])
            .await;
            let mut storage = new_storage(flash).await;

            let mut data = KeymapData::new([[[KeyAction::No]]]);
            let mut behavior = BehaviorConfig::default();
            storage.read_keymap(&mut data, &mut behavior).await.unwrap();

            let positional = crate::config::PositionalConfig::<1, 1>::default();
            let keymap = KeyMap::new(&mut data, &mut behavior, &positional).await;

            assert_eq!(keymap.layout_option(), 42);
        });
    }
}
