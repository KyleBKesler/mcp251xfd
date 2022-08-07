# mcp251xfd

This is a partial userspace driver for mcp251xfd can controller chips. Its made for use with Mike McCauley's bcm2835 spi driver for raspberry pi.
The instructions on installing that driver are documented here: https://www.airspayce.com/mikem/bcm2835/

The main goal of this project was to be able to control the states of the mcp251xfd controller and apply specific settings for the use of the 2 optional INT/GPIO/STDBY pins.

I've organized the driver into 4 main sections:

SPI Access Functions
  - SPI initialazation functions
  - read/write byte
  - read/write word

MCP251xFD Operation Mode Controls
  - get operation mode
  - set operation modes
  - optional wait for mode change

Wakeup Controls
  - setting wakeup filter options
  - enabling wakeup interrupt

INT/GPIO/STDBY Control
  - various settings for use of optional pins

*some functions have been made static to avoid misuse*
