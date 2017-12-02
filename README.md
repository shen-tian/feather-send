# Tracker code

Ported over to PlatformIO from [electrosew](https://github.com/mrgriscom/electrosew/blob/master/feather_send/feather_send.ino). This is the multi-agent version. It's configured for a Feather M0 based board, but should run fine on a 32u4 Feather too.

## Setup

Install `platformio`. Tested using 3.4.1.

## Libs used:

- Adafruit GFX
- RadioHead
- SSA1306
- TinyGPS

Should get installed automatically.

Then, to build, go

    > pio run

To upload

    > pio --target upload

## IDE setup

Atom/VSCode something. Who knows.

## Emacs setup

`package-install` `platform-io` mode. Not clear what this should hook onto
so enable it manually in the `main.cpp` buffer. (Assume we don't want
this for all `.cpp` files?) This also need `Projectile`, and the code
need to be in a repo, so it can find the root folder.

Once that's done, `C-c i b` to build, and `C-c i u` to upload.

* Potential issue: *

Had some funny colors in the `*compilation*` buffer.

````elisp
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
