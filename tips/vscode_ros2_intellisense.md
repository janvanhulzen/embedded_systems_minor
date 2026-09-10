# ROS 2-headerbestanden vindbaar maken in VS Code op Ubuntu

Wanneer onder een regel zoals onderstaande een rode golvende lijn staat, kan de C/C++-extensie van VS Code het headerbestand niet vinden:

```cpp
#include <rclcpp/rclcpp.hpp>
```

Deze instructie laat zien hoe je het zoekpad van IntelliSense instelt. De instellingen komen in `.vscode/c_cpp_properties.json` te staan.

> **Let op:** de rode lijn is een melding van IntelliSense, de code-analyse van VS Code. Als `colcon build` wel slaagt, is de ROS 2-code correct geconfigureerd en betreft het alleen de editor. Als `colcon build` ook faalt, moet mogelijk een ROS 2-package worden geïnstalleerd of `CMakeLists.txt` worden aangepast.

## 1. Open de volledige workspace in VS Code

Open een terminal en ga naar de hoofdmap van de ROS 2-workspace:

```bash
cd ~/ros2_ws
```

Laad ROS 2 en, als de workspace al is gebouwd, ook de lokale workspace:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Open daarna vanuit deze terminal de volledige directory in VS Code:

```bash
code .
```

Als `install/setup.bash` nog niet bestaat, kan die tweede `source`-opdracht worden overgeslagen.

## 2. Controleer de C/C++-extensie

1. Open in VS Code **Extensions** met `Ctrl+Shift+X`.
2. Zoek naar:

   ```text
   @id:ms-vscode.cpptools
   ```

3. Controleer of de extensie **C/C++** van Microsoft is geïnstalleerd en ingeschakeld.

De zichtbare naam van de extensie is **C/C++**. `ms-vscode.cpptools` is de technische extensie-ID.

## 3. Zoek het ontbrekende headerbestand

Open in VS Code een terminal met `` Ctrl+` `` en zoek naar `rclcpp.hpp`:

```bash
find /opt/ros/jazzy -name rclcpp.hpp 2>/dev/null
```

Een gebruikelijke uitkomst is:

```text
/opt/ros/jazzy/include/rclcpp/rclcpp/rclcpp.hpp
```

Bij deze include-opdracht:

```cpp
#include <rclcpp/rclcpp.hpp>
```

moet IntelliSense dus zoeken binnen de include-directories onder:

```text
/opt/ros/jazzy/include
```

De toevoeging `/**` in de VS Code-configuratie zorgt ervoor dat ook onderliggende directories worden doorzocht.

## 4. Open de IntelliSense-configuratie

1. Druk in VS Code op `Ctrl+Shift+P` om het **Command Palette** te openen.
2. Typ:

   ```text
   C/C++: Edit Configurations (JSON)
   ```

3. Selecteer deze opdracht.

VS Code opent of maakt vervolgens:

```text
.vscode/c_cpp_properties.json
```

## 5. Voeg het ROS 2-include-pad toe

Zoek in het JSON-bestand naar `includePath` en voeg hieraan toe:

```json
"/opt/ros/jazzy/include/**"
```

Een complete basisconfiguratie ziet er dan als volgt uit:

```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/jazzy/include/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/g++",
            "cStandard": "c17",
            "cppStandard": "c++17",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
```

Staat er al een configuratie in het bestand, vervang die dan niet zonder meer. Voeg alleen het ontbrekende pad toe aan de bestaande `includePath`-lijst. Let daarbij op de komma's tussen de regels; JSON accepteert geen komma achter het laatste element.

## 6. Vernieuw IntelliSense

1. Sla `c_cpp_properties.json` op met `Ctrl+S`.
2. Open opnieuw het Command Palette met `Ctrl+Shift+P`.
3. Voer uit:

   ```text
   C/C++: Reset IntelliSense Database
   ```

De rode golvende lijn onder `#include <rclcpp/rclcpp.hpp>` hoort nu te verdwijnen. Als dat niet direct gebeurt, voer dan via het Command Palette ook het volgende uit:

```text
Developer: Reload Window
```

## Als het headerbestand niet wordt gevonden door `find`

Controleer of `rclcpp` is geïnstalleerd:

```bash
dpkg -L ros-jazzy-rclcpp | grep rclcpp.hpp
```

Als het package ontbreekt, installeer het met:

```bash
sudo apt update
sudo apt install ros-jazzy-rclcpp
```

Gebruik bij een andere ROS 2-distributie de bijbehorende naam en directory, bijvoorbeeld `humble` in plaats van `jazzy`.

## Als `colcon build` ook een fout geeft

Controleer dan of in `CMakeLists.txt` ten minste staat:

```cmake
find_package(rclcpp REQUIRED)
```

Koppel `rclcpp` bovendien aan de betreffende executable:

```cmake
ament_target_dependencies(mijn_node rclcpp)
```

Vervang `mijn_node` door de naam die bij `add_executable(...)` is opgegeven.

## Achtergrond

`c_cpp_properties.json` configureert de C/C++-extensie van VS Code. Het bestand bepaalt onder andere welke compiler, taalstandaard en include-directories IntelliSense gebruikt. Het verandert niet automatisch de buildconfiguratie van CMake of ROS 2.

Meer informatie: [C++ extension settings reference van Visual Studio Code](https://code.visualstudio.com/docs/cpp/customize-cpp-settings).
