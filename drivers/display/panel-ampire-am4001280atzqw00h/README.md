# Ampire AM-4001280ATZQW-00H MIPI-DSI panel driver

Display driver for the AM-4001280ATZQW-00H panel series by [Ampire](http://www.ampire.com.tw/).

## Compatibility

### Warning: software unstable and in development

The driver is being tested on the [Verdin iMX8M Mini](https://developer.toradex.com/hardware/verdin-som-family/modules/verdin-imx8m-mini/) attached to a [Dahlia V1.1 carrier board](https://developer.toradex.com/hardware/verdin-som-family/carrier-boards/dahlia-carrier-board/) by [Toradex](https://www.toradex.com/de) with a custom MIPI adapter.
Here, the device was registered via an overlay which is available [here](https://github.com/j-gre/overlays/tree/main/toradex/verdin-imx8mm_am4001280atzqw00h.dts).

Other systems may require small alterations to the device tree and driver.

## Deployment

Use the makefile to compile the driver as a module for Yocto builds: 


## License

GPL-2.0-only
