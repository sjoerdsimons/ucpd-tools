# USB-C PD analyzer tooling

** Very early **

Three parts:

* stm32g071b-disco-firmware: Replacment firmware for
  the [STM32G071B-DISCO board](https://www.st.com/en/evaluation-tools/stm32g071b-disco.html)
* analyzer-cbor: Helper crate for cbor serialization of capture data
* ucpd-capture: Capture data output by the stm over "virtual" serial and do PD parsing

TL;DR reflash the disco board with the firmare via ST-Link (built-in, on
usb-micro port); Use ucpd-capture to the data from the expose USB serial port
and see PD packets fly when plugging the disco board between PD-capable
devices.
