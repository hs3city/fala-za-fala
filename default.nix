{ pkgs ? import (
  builtins.fetchTarball {
    url = "https://github.com/nixos/nixpkgs/archive/22.11.tar.gz";
    sha256 = "11w3wn2yjhaa5pv20gbfbirvjq6i3m7pqrq2msf0g7cv44vijwgw";
  }
) {} }:

with pkgs;

mkShell {
  buildInputs = [
    arduino-cli
    clang-tools
    pre-commit
  ];
  shellHook = ''
    arduino-cli config init || true
    arduino-cli core update-index
    arduino-cli config add board_manager.additional_urls https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
    arduino-cli core install Seeeduino:mbed
    arduino-cli lib install "Adafruit NeoPixel"@1.11.0
    arduino-cli lib install arduinoFFT@1.6.0
    pre-commit install
  '';
}
