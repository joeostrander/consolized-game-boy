# consolized-game-boy

## Description
This is a modified version of Andy West's project from *element14 Presents* episode 531: "Game Guy - The Unportable Game Boy".  

![gameplay](./images/gameplay.gif?raw=true)  

![front closed](./images/front_closed.jpg?raw=true)  

![front open](./images/front_open.jpg?raw=true)  

![back](./images/back.jpg?raw=true)  

![screencap osd](./images/screencap_osd.jpg?raw=true)  

![screencap 1](./images/screencap1.jpg?raw=true)  

![screencap 2](./images/screencap2.jpg?raw=true)  


Notable changes:
 - Single Pico!
 - Using RGB222 or RGB332 Video to free up GPIO (with the limited color schemes, I doubt anyone would notice!)
 - All pins have been remapped; this was done mostly to make things easier to route PCB traces
 - 3x scale maintaining aspect ratio
 - Option to enable scanlines, pixel effect, etc.
 - The Color scheme, border, effects can be changed via a menu overlay
 - MODEL_B version fits inside a NESPi case, with SGB Cart rev, the cartridge fits all the way in
 - Custom PCBs
 - With MODEL_B PCB, you won't need a functioning Gameboy motherboard.  You just need a good CPU and RAM (you could even use CPU/RAM from a Super Gameboy SNES/Famicom cartridge)
 - There is now an RGB332 version, currently in a separate branch (RGB332) because of the GPIO pinout changes  

![preview](https://github.com/joeostrander/consolized-game-boy/blob/main/images/preview.png?raw=true)

[Video sample](https://youtu.be/khdu8cWNxHo)

[OSD](https://youtu.be/VJUn-7w2_1k)

Andy liked my modifications and made an updated video where he built one with my changes :)
[![The Game Guy Mini, Upgrading the Unportable Game Boy!](https://i.ytimg.com/vi/gPNHySf-hk0/0.jpg)](https://youtu.be/gPNHySf-hk0)

Check out Andy's original video!  
[![Game Guy - The Unportable Game Boy](https://img.youtube.com/vi/ypGMU5lLjeU/0.jpg)](https://www.youtube.com/watch?v=ypGMU5lLjeU)
 
element14 Community pages:
 - https://community.element14.com/challenges-projects/element14-presents/project-videos/w/documents/27407
 - https://community.element14.com/w/documents/27862/episode-577-the-game-guy-mini-upgrading-the-unportable-game-boy

