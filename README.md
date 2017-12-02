# Tracker code

Ported over to PlatformIO from [electrosew](https://github.com/mrgriscom/electrosew/blob/master/feather_send/feather_send.ino). This is the multi-agent version. It's configured for a Feather M0 based board, but should run fine on a 32u4 Feather too.

## Setup

Install `platformio`. Tested using 3.4.1.

## Libs required:

- 13 : Adafruit GFX
- 124 : RadioHead
- 135 : SSA1306
- 416 : TinyGPS

To install them, go

    platformio lib install 13
    platformio lib install 124
    platformio lib install 135
    platformio lib install 416

Then, to build, go

    > pio run

To upload

    > pio --target upload

## Emacs setup

`package-install` `platform-io` mode. Not clear what this should hook onto
so enable it manually in the `main.cpp` buffer. This also need `Projectile`
to be installed, so it can find the root folder (thus also need this to
be a git repo).

Once that's done, `C-c i b` to build, and `C-c i u` to upload.

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
