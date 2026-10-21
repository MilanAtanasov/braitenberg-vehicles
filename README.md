# Braitenberg vehicles 1–14

A static web page that simulates the fourteen vehicles from Valentino Braitenberg, *Vehicles: Experiments in Synthetic Psychology* (MIT Press, 1984). Each vehicle comes with a few variants and sliders for its parameters. An "Inside the vehicle" panel shows its wiring and live signals. `playground.html` puts several vehicles into one arena with food that appears and expires.

## Open

There is no build step and nothing to install. Open `index.html` in a browser, or serve the folder with a static server such as `python3 -m http.server`.

## URL options

- `index.html#3c` selects vehicle 3, variant c.
- `?seed=N` fixes the start of the run so it can be reproduced. The address updates when you press "New start".
- `?speed=N` sets the starting speed. The slider runs from 0.25 to 20, and choosing another vehicle resets it to that vehicle's default.
- `?present=1` starts in presentation mode.
- `?embed=1` shows the stage only, for embedding in an iframe.

## Keys

- Space: pause or play
- R: reset the run
- ← and →: previous and next vehicle
- P: presentation mode on or off
- I: show or hide the inner panel below the stage (presentation mode only)
- Esc: leave presentation mode

## Recording

The "Save image" and "Record clip" buttons use the select next to them:

- Stage and panel: 1920×1584 (stage on top, inner panel below)
- Stage only: 1920×1080

The clip is saved as .webm in Chrome, Edge or Firefox. Keep the tab visible while recording. To make an MP4 for PowerPoint, run `./make-mp4.sh clip.webm` (needs ffmpeg).
