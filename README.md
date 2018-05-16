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

`package-install` `platformio-mode`. I'm grabbing it from Melpa Stable.
This also need `Projectile`, and the code need to be in a repo, so it can
find the root folder.

The following snippest allows you to automatically load `platformio-mode`.

```elisp
(defun platformio-conditionally-enable ()
  "Enable `platformio-mode' only when a `platformio.ini' file is present in project root."
  (condition-case nil
      (when (projectile-verify-file "platformio.ini")
        (platformio-mode 1))
    (error nil)))

;; Enable irony for all c++ files, and platformio-mode only
;; when needed (platformio.ini present in project root).
(add-hook 'c++-mode-hook (lambda ()
                           (platformio-conditionally-enable)))
```

Once that's done, `C-c i b` to build, and `C-c i u` to upload.

* Potential issue: *

Had some funny colors in the `*compilation*` buffer.

```elisp
(add-hook 'eshell-preoutput-filter-functions
           'ansi-color-filter-apply)

(require 'ansi-color)
(defun colorize-compilation-buffer ()
  (toggle-read-only)
  (ansi-color-apply-on-region compilation-filter-start (point))
  (toggle-read-only))
(add-hook 'compilation-filter-hook 'colorize-compilation-buffer)
```

this seems to have helped?
