<div align="left">
  <h1>📡 ESP-GRABER   <a href="#ru">Русский</a> | <a href="#en">English</a></h1>

![ESP-GRABER_LOGO](https://github.com/user-attachments/assets/79845259-5289-4785-bf46-4f89b9422fe2)

</div>

<div id="ru">
  
## 🚀 О проекте ESP-GRABER
ESP-GRABER — универсальный инструмент для работы с радиочастотами на базе ESP32.  
Проект позволяет **считывать**, **повторять** и **сохранять** сигналы в диапазонах 315/433 МГц.  
*Проект стабилен, но может дорабатываться для расширения функционала.*

### ⚠️ Дисклеймер
Данная прошивка разработана исключительно для исследовательских целей и тестирования оборудования. 
Используя прошивку вы обязаны учитывать законодательство своего региона. Создатель прошивки не несет ответственность за ваши действия.                            
*Использование прошивки означает ваше полное согласие с этими условиями*  
## ⚡ Возможности
### 📶 SubGHz (315/433 МГц)
- 🎯 **Считывание сигналов**
- 🔄 **Повтор сигналов**
- 💾 **Хранение сигналов**

## 🛠️ Сборка
### 🔧 Необходимые компоненты
| Компонент | Ссылка |
|-----------|--------|
| ESP32-WROOM | [AliExpress](https://aliexpress.ru/item/1005004605399313.html) |
| CC1101-Модуль | [AliExpress](https://aliexpress.ru/item/1005008544032996.html) |
| Дисплей&кнопки | [AliExpress](https://aliexpress.ru/item/1005006322355552.html) |
| Макетная плата | [AliExpress](https://aliexpress.ru/item/1005008466693134.html) |
| Провода-перемычки | [AliExpress](https://aliexpress.ru/item/1005007553381854.html) |

### 🔌 Схема подключения
![Схема](https://github.com/user-attachments/assets/26730497-8100-4cc1-8361-187221489662)
|Module|Pin 1|Pin 2|Pin 3|Pin 4|Pin 5|Pin 6| Pin 7|
|--------|--------|--------|--------|--------|--------|--------|--------|
|**📺 Display**|VCC → 3V3|GND → GND|SCL → G22|SDA → G21|-|-|-|
|**🔘 Buttons**|K1 → G27|K2 → G26|K3 → G33|K4 → G32|-|-|-|
|**📡 CC1101**|1 → GND|2 → 3V3|3 → G2|4 → G5|5 → G18|6 → G23 |7 → G19|

### 📸 Финальный результат
![ESP-GRABER Device](https://github.com/user-attachments/assets/e881e9e7-5e73-4fd2-a1cd-2f0002cca44b)

</div>

<div id="en" hidden>

## 🚀 About ESP-GRABER
ESP-GRABER is a versatile tool for working with radio frequencies based on ESP32.  
The project allows **reading**, **repeating**, and **saving** signals in the 315/433 MHz ranges.  
*The project is stable but may be updated for additional features.*

### ⚠️ Disclaimer
This firmware is designed exclusively for research purposes and hardware testing. 
When using the firmware, you must take into account the laws of your regio. The firmware creator is not responsible for your actions.                            
*Using the firmware means that you fully agree to these terms*     

## ⚡ Features
### 📶 SubGHz (315/433 MHz)
- 🎯 **Signal Grabbing**
- 🔄 **Signal Replay**
- 💾 **Signal Storage**

## 🛠️ Building
### 🔧 Required Components
| Component | Link |
|-----------|------|
| ESP32-WROOM | [AliExpress](https://aliexpress.ru/item/1005004605399313.html) |
| CC1101 Module | [AliExpress](https://aliexpress.ru/item/1005008544032996.html) |
| Display & Buttons | [AliExpress](https://aliexpress.ru/item/1005006322355552.html) |
| Breadboard | [AliExpress](https://aliexpress.ru/item/1005008466693134.html) |
| Jumper Wires | [AliExpress](https://aliexpress.ru/item/1005007553381854.html) |

### 🔌 Connection Scheme
![Scheme](https://github.com/user-attachments/assets/26730497-8100-4cc1-8361-187221489662)
|Module|Pin 1|Pin 2|Pin 3|Pin 4|Pin 5|Pin 6| Pin 7|
|--------|--------|--------|--------|--------|--------|--------|--------|
|**📺 Display**|VCC → 3V3|GND → GND|SCL → G22|SDA → G21|-|-|-|
|**🔘 Buttons**|K1 → G27|K2 → G26|K3 → G33|K4 → G32|-|-|-|
|**📡 CC1101**|1 → GND|2 → 3V3|3 → G2|4 → G5|5 → G18|6 → G23 |7 → G19|

### 📸 Final Result
![ESP-GRABER Device](https://github.com/user-attachments/assets/e881e9e7-5e73-4fd2-a1cd-2f0002cca44b)

</div>
