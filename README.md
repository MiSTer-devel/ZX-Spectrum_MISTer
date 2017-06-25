# ZX Spectrum 128K for [MiSTer Board](https://github.com/MiSTer-devel/Main_MiSTer/wiki)

Some verilog models from Till Harbaum [Spectrum](https://github.com/mist-devel/mist-board/tree/master/cores/spectrum) core were used in this project.

### Features:
- Fully functional ZX Spectrum 48K, 128K and Pentagon 128 with correct CPU and Video timings.
- Pentagon 256K and Scorpion 256K extended memory interfaces.
- Turbo 7MHz, 14MHz, 28MHz, 56MHz.
- ULA+ v1.1 programmable palettes with extended Timex control.
- Timex HiColor, HiRes modes.
- Original Tape loading through OSD (CSW files).
- TR-DOS (Beta Disk Interface) and native TRD images.
- G+DOS (MGT +D Disk Interface) and IMG, MGT images.
- Multiface 128 add-on.
- Memory snapshot save/load in +D and Multiface.
- Native TAP with turbo loading. Fast loading for TAP and CSW.
- Kempston Mouse and Joystick.
- Audio in from real [tape device](http://www.atari-forum.com/viewtopic.php?p=298401#p298401)

### Installation:
Copy the *.rbf file at the root of the SD card.
Copy [spectrum.rom](https://github.com/MiSTer-devel/ZX-Spectrum_MISTer/tree/master/rom) file to the root of SD card.
**Note:** always update spectrum.rom together with core to make sure you're using compatible ROM version. ROM is not always compatible with all releases (but always compatible with latest release), thus you need to keep the ROM if you want to use older version of core.

### Notes about supported formats:
**TRD** is TR-DOS image used with Beta Disk Interface (BDI). To use TR-DOS you need to choose TRD image in OSD first. In 128K mode use menu to enter TR-DOS. In 48K mode use command **RANDOMIZE USR 15616** to enter TR-DOS. Use command **RETURN** to leave TR-DOS.

**IMG** is G+DOS image used with +D Disk interface. Although it's fully supported, i couldn't find any games on such disks. The main purpose of these images is to use snapshot function of +D and Multiface.

**MGT** is G+DOS and MasterDOS (SAM Coupe) image. It's similar to IMG but uses different layout. The main purpose is to transfer data to/from SAM Coupe.

**TAP** is simple tape dump format. It is possible to use normal and **turbo** loading (only if application uses standard loading routines from ROM). To load in turbo mode, you need to choose TAP file in OSD **first** and then start to load app through menu (128K) or by command **LOAD ""** (48K, 128K). To load TAP file in normal mode through internal AUDIO IN loop, you need to start loading through menu or command **first** and then choose TAP file though OSD. If application uses non-standard loader, then TAP file will be played in normal mode automatically. Thus it's safe to always choose the turbo mode. Some applications are split into several parts inside one TAP file. For example DEMO apps where each part is loaded after finish of previous part, or games loading levels by requests. The core pauses the TAP playback after each code part (flag=#255). If application uses standard loader from ROM, then everything will be handled automatically and unnoticeable. If app uses non-standard loader, then there is no way to detect the loading. In this case you need to press **F1 key** to continue/pause TAP playback. Do not press F1 key while data is loading (or you will have to reset and start from beginning). To help operate with TAP (for non-standard loaders) there is special yellow LED signaling:
- LED is ON: more data is available in TAP file.
- LED is flashing: loading is in process.
- LED is OFF: no more data left in TAP file.

In normal mode, while TAP loading, the following keys can be used:
- F1 - pause/continue
- F2 - jump to previous part (if pressed while pilot tone), or beginning of current part (if pressed while code is transferring).
- F3 - skip to next part

**CSW** is tape format, useful only for apps using non-standard loaders with non-standard transfer speeds. These files are are always loaded in normal mode. You can use **F1** key to pause/continue.

OSD option **Fast tape load** increases CPU frequency to 56MHz while tape loading.

Use **F10** key to switch into 48K basic (won't lock 48K mode) and automatically enter **LOAD ""** if game/app doesn't load from 128K menu.

### Turbo modes
You can control CPU speed by following keys:
- F4 - normal speed (3.5MHz)
- F5 - 7MHz
- F6 - 14MHz
- F7 - 28MHz
- F8 - 56MHz
- F9 - pause/continue

It's useful to switch to maximum speed when you are loading tape in normal mode. Due to SDRAM speed limitation 28MHz and 56MHz speeds include wait states, so effective CPU speed is lower than nominal.

### Memory Configurations with extra RAM:
- **Pentagon 256K** uses bit 6 in port 7FFD to access additional memory.
- **Scorpion 256K** uses bit 4 in port 1FFD to access additional memory, allows to use RAM in 0-3FFF (bit 0 in 1FFD)

### Mouse and Joystick:
Kempston mouse has no strict convention which bit (D0 or D1) reflects a main button. After each reset, the first button pressed on mouse (left or right buttons only) will be represented by bit D0 (other button will be represented by bit D1). So, if you are not satisfied by mouse button map, then simply press reset and then press other button first.
Due to port conflict with Kempston joystick, core uses autodetection. Any mouse activity will switch port to mouse control. Any joystick activity will switch port to joystick control.
Some games/apps autodetect the mouse. So, move the mouse or click its button before use such games/apps.

### Snapshots:
Core supports snapshot functionality of +D. In order to use it, you need to mount IMG or MGT image. ROM includes preloaded G+DOS image, thus you can mount IMG/MGT at any time (even while playing the game). **Note #1**: preloaded G+DOS has been patched to allow disk change on-the-fly. So, if you will load G+DOS from disk, then be careful - it may corrupt previous saves if you will change the disk! **Note #2:** only one disk image can be mounted at any time. Thus make sure if you use game from TRD image, the game won't save anything later to its disk. 

To save snapshot using +D (preferred way), press **F11 key**. You will see stripes on border and game will freeze. You can press following keys:
- 3 - to save the screen.
- 4 - to save 48K snapshot.
- 5 - to save 128K snapshot.
Original +D ROM requires to press additional Y/N keys in 128K mode to choose the correct screen buffer. Included ROM has been pre-patched to automatically detect the screen. So, just press 5 for 128K snapshot.

To load snapshot, just mount IMG/MGT and go to basic prompt where type **CAT 1** to list its content. Note the number of snapshot file. Then type **LOAD pX** where X is the number of shapshot file. For other disk commands please find and read G+DOS (MGT +D) manual.

### Multiface 128:
You can enter Multiface ROM using **RShift+F11**. Multiface 128 includes preloaded debugger where you can trace or modify the game. If you prefer to use bare Multiface 128 ROM then do following procedure: Press and hold **ESC**, then press **RShift+F11**. You will be able to use bare Multiface ROM by simple subsequent presses of **RShift+F11** till core reload. Multiface provides snapshot functionality by saving to IMG/MGT disks. Please find and read Multiface 128 manual. **Note:** Multiface 128 expose its port, thus if game has protection against Multiface, it won't work. The port will start to be exposed after FIRST press of **RShift+F11** till reset. Thus using +D snapshot is prefered.

### Special Keys:
- Ctrl+F11 - warm reset
- Alt+F11 - cold reset will disk unload
- Ctrl+Alt+F11 - reset to ROM0 menu
- F11 - enter +D snapshot menu (or ROM0 menu if IMG/MGT not mounted)
- RShift+F11 - enter Multiface 128 menu
- F12 - OSD menu

### Download precompiled binaries and system ROMs:
Go to [releases](https://github.com/MiSTer-devel/ZX-Spectrum_MISTer/tree/master/releases) folder.
