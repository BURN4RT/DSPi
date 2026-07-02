# Linux PipeWire Card Profile

The DSPi is a USB Audio Class device with a physical S/PDIF output. Without a
DSPi-specific card profile, PipeWire can expose it as two outputs (analog and
digital) and the digital output volume control may not work correctly.

The files in `tools/linux-pipewire-card-profile/` install a card profile, mixer path, and udev
rule that make PipeWire use a single `Digital Stereo (IEC958)` profile for the
DSPi and expose the USB Audio hardware volume control on the digital output.

> **PipeWire only.** This relies on PipeWire's ACP mechanism (the
> `ACP_PROFILE_SET` udev property and the `~/.config/alsa-card-profile` search
> path). It does not take effect under the classic PulseAudio daemon, which uses
> a different udev property (`PULSE_PROFILE_SET`) and config directory. PipeWire
> is the default audio server on most current mainstream desktops, including
> Fedora, Ubuntu, Pop!_OS, Linux Mint, Debian and openSUSE Tumbleweed, and is
> the standard choice on Arch.

Install the profile:

```sh
./install-dspi-profile.sh
```

The installer:

- copies the profile and mixer path into your user config directory
  (`$XDG_CONFIG_HOME/alsa-card-profile/mixer`, no root required). PipeWire's ACP
  loader reads this in addition to `/usr/share`, so it also works on immutable
  ostree systems such as Fedora Silverblue/Kinoite where `/usr/share` is
  read-only;
- installs `/etc/udev/rules.d/91-dspi-pipewire-alsa.rules` (needs root), reloads
  udev, and retriggers the DSPi sound card if it is connected;
- restarts the user audio services when possible.

If you run the installer inside a Toolbx container, the udev steps are relayed
to the host with `flatpak-spawn --host` (you authenticate once via polkit),
since udev lives on the host rather than in the container.

Useful installer options:

```sh
./install-dspi-profile.sh --dry-run
./install-dspi-profile.sh --no-restart
```

After installation, unplug and reconnect the DSPi if it does not immediately
appear as a single digital output.

Remove the profile:

```sh
./uninstall-dspi-profile.sh
```
