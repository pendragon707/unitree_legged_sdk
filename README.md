# v3.8.4
The aliengo_sdk is mainly used for communication between PC and aliengo control board.
It also can be used in other PCs with UDP.

### Notice
support robot: Aliengo

not support robot: Laikago, Aliengo, A1. (Check release [v3.3.1](https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/v3.3.1) for support)

## Как настроить коннект между роботом и компьютером + првоерка этого соединения:

1.Установим библиотеки :
 
1. 1-Установим git
```
sudo apt-get install git
```
1. 2-Установим Glib and Cmake
```
sudo apt update && sudo apt install build-essential g++ libglib2.0-dev cmake  libboost-all-dev libmsgpack*
```

2.Установим lcm >=1.4.0:
```
wget https://github.com/lcm-proj/lcm/archive/refs/tags/v1.5.0.zip && unzip v1.5.0.zip && mv lcm-1.5.0 lcm
```

2. 1 - Созадем папку build
```
cd lcm
mkdir build
cd build
```
2. 2- Cmake
```
cmake ..
```
2. 3 - Устанавливаем make
```
make
sudo make install
```
2. 4- Устанавливаем python dev
```
 sudo apt-get install python-dev && sudo apt-get install python3-dev
```
2. 5- Устанавливаем LCM
```
cd ..
cd lcm-python
sudo python3 setup.py install
```
3.Клонируем репозиторий с sdk (на примере робота Aliengo)
```
git clone https://github.com/unitreerobotics/unitree_legged_sdk.git -b Aliengo
```
3. 1 - Устанавливаем сам sdk
```
cd unitree_legged_sdk
mkdir -p build
cd build
cmake ..
make 
```
3. 2 - Настраиваем сеть

Если адрес порта автоматически был выставлен неверно, то устанавливаем Network->Wired->IPv4->Manual адрес 192.168.123.200 с маской 255.255.255.0

3. 3 - Проверка соединения - Ping
```
ping 192.168.123.10
```
4.Запуск sdk на реальном роботе
```
cd unitree_legged_sdk
cd build
sudo ./example_torque_aliengo # (например)
```
5. Если при запуске возникли  ошибки по типу:
```
liblcm.so.1: cannot open shared object file: No such file or directory
```
Ипользуйте в терминале комманду :
```
sudo ldconfig -v
```

### Если вам нужен интерфейс для общения на языке программирования Python

1. Заходите в папку python_wrapper
```
cd Aliengo/python_wrapper
```
2. Создаете папку build и cкомпилируете robot_interface_aliengo (если есть желание изменить название этого файла перейди в файл python_wrapper/python_interface.cpp строка 21: PYBIND11_MODULE(robot_interface_aliengo, m))
```
mkdir build
cd build
cmake ..
make 
```
3. На всякий случай проверьте папку Aliengo/lib/python/amd64/robot_interface... и убедитесь что файл создан , у меня он выглядел так при условии что окружение было python3.10 (robot_interface_aliengo.cpython-310-x86_64-linux-gnu.so) в вашем случае все будет зависеть от вашей версии python

4. При условии что все создалось и скомпилировалось можно попробовать запустить пример на python. Для этого перейдите в папку Aliengo/example_py/example_torque.py и убедитесь в правильности названия (совпадения) имен robot_interface в файле python и скомпиленном файле. В моем случае скомпилированный файл называется robot_interface_aliengo и соответсвенно в файлике  python , 9-ая строка выглядит так : import robot_interface_aliengo as sdk
```
python3 example_torque.py
```


### Sport Mode
```bash
Legged_sport    >= v1.0.20
firmware H0.1.7 >= v0.1.35
         H0.1.9 >= v0.1.35



### Dependencies
* [Boost](http://www.boost.org) (version 1.5.4 or higher)
* [CMake](http://www.cmake.org) (version 2.8.3 or higher)
* [LCM](https://lcm-proj.github.io) (version 1.4.0 or higher)
```bash
cd lcm-x.x.x
mkdir build
cd build
cmake ../
make
sudo make install
```

### Build
```bash
mkdir build
cd build
cmake ../
make
```

### Run

#### Cpp
Run examples with 'sudo' for memory locking.

#### Python
##### arm
change `sys.path.append('../lib/python/amd64')` to `sys.path.append('../lib/python/arm64')`
