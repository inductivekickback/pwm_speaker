This is a Zephyr RTOS driver for playing PCM sound buffers and tones on the [Thing:52](https://www.nordicsemi.com/Products/Development-hardware/Nordic-Thingy-52). It was built from the v1.6.1 tag of the [nRF Connect SDK (NCS)](https://github.com/nrfconnect/sdk-nrf).

### About the PCM sound buffers
This driver plays 64kbps (8-bit, 8KHz) PCM sound buffers (from either flash or RAM). It has been tested with the original [Thingy:52 firmware's sounds](https://nordicsemiconductor.github.io/Nordic-Thingy52-FW/documentation/sounds_8h_source.html).

### Using the driver
This is the example DT entry in the project's local overlay (e.g. "thingy52_nrf52832.overlay"):
```
thingy_speaker: nrf-speaker0 {
    compatible = "nordic,nrf-sw-speaker";
    status = "okay";
    enable_pin = <29>;
    output_pin = <27>;
    label = "thingy_speaker";
};
spk-pwr-ctrl {
    status = "disabled";
};
```
Then add the following to **prj.conf**:
```
CONFIG_SPEAKER=y
```
Tones can be played:
```
int err = speaker_tone_play(dev, 400 /*Hz*/, 500 /*ms*/, 100/*volume*/, speaker_callback);
```
And PCM buffers can be played:
```
int err = speaker_pcm_play(dev, sound_0, sizeof(sound_0), speaker_callback);
```
