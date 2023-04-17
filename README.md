# Keil Template for STC32G .Lib Ver

Having a Happy travel with STC32!  : (

(I do trust STC32 is the best MCU in the earth)

---

## How to use it
### initialize project
```shell
git clone https://github.com/msuadOf/STC32-Keil-Template 01_STC32G_app
cd 01_STC32G_app
python projectname_init.py
```
or
```shell
git clone https://github.com/msuadOf/STC32-Keil-Template 01_STC32G_app
python ./01_STC32G_app/projectname_init.py
```

**if you do not have python in you PC,you can do it manually**

1. put your app code in `CORE/`and your lib files in `Driver/`
2.  then open `01_GPIO.uvproj`
3. change name of the project: the name of keil file`01_GPIO.uvproj`, `Option for Target-->Output-->Name of Executable`  and  `Manage Project Items-->Project Targets`
4. enjoy it !