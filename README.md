# Tracker code

Ported over to PlatformIO from [electrosew](https://github.com/mloudon/electrosew/tree/master/feather_send)

## Libs required:

- 13 : Adafruit GFX
- 124 : RadioHead
- 135 : SSA1306
- 416 : TinyGPS

To install them, go

    platformio lib install 13
    platformio lib install 124

etc.

## Emac setup

Had some funny colors in the `*compilation*` buffer.

````
(add-hook 'eshell-preoutput-filter-functions
           'ansi-color-filter-apply)

(require 'ansi-color)
(defun colorize-compilation-buffer ()
  (toggle-read-only)
  (ansi-color-apply-on-region compilation-filter-start (point))
  (toggle-read-only))
(add-hook 'compilation-filter-hook 'colorize-compilation-buffer)
````

this seems to have helped?

Otherwise, install `platformio-mode` from MELPA
