import re
import os

projectname_old = "01_GPIO"
projectname_new = ""


path = os.path.dirname(__file__)  # file dir
# print(os.path.abspath('.')) #work dir
dirStr, ext = os.path.splitext(path)
file = dirStr.split("\\")[-1]
print("\n====Your new project name will be \""+file+"\"====\n")

projectname_new = file  # Update project name



def alter(file, old_str, new_str):
    """
    Replace string in file
    :param file:
    :param old_str:
    :param new_str:
    :return:
    """
    file_data = ""
    with open(file, "r", encoding="utf-8") as f:
        for line in f:
            if old_str in line:
                line = line.replace(old_str, new_str)
            file_data += line
    with open(file, "w", encoding="utf-8") as f:
        f.write(file_data)


# ==== operation start ! ====
try:
    # change content of keil file from dir name
    alter(f"./{projectname_old}.uvproj",
          f"{projectname_old}", f"{projectname_new}")
    print(f"[\033[32mSUCCESS\033[0m] \"./{projectname_old}.uvproj\" content have been changed")

    # change keil file name
    os.rename(f"./{projectname_old}.uvproj", f"./{projectname_new}.uvproj")
    print(f"[\033[32mSUCCESS\033[0m] ./{projectname_old}'s name have been changed into ./{projectname_new}")

    print("\n\n     \033[32mEnjoy your STC32!\033[0m")
except FileNotFoundError:
    print(
        f"\033[31m[Error]: Please do not change original name of kei file,and change it back to \"{projectname_old}.uvproj\"\033[0m")
except:
    print("\033[31m[Error]: An error we'd never known has orrured,you can contact me by https://github.com/msuadOf/STC32-Keil-Template/issues\033[0m")




# delete keil working file

def getfilelist(path, strn, callback=lambda image_name: 0):
    names = []
    for dirpath, dirnames, filenames in os.walk(path):
        for filepath in filenames:
            #             image_name=os.path.join(dirpath, filepath)#鑼呰伃鍟毊鑹傛幊鏆椀鑱¤寘鑱仠鏆伕鑹㈣啺娈磋棸鑼呰伂绡撴毊鑱ユ壄鏆伀鑱幗鑱墽鍗仮鑱疯幗钘濊伝
            image_name = filepath
            str1 = re.compile(strn+'''(.*?).uv*''')
            match_obj = re.findall(str1, image_name)
            if match_obj:
                # print(image_name)
                callback(image_name)
                names.append(image_name)
#             shutil.copy(image_name,  targetDir) #鑶板暘鑹ｆ毊鑹ｈ仧鏆伡鑱磋寘鑱ц伄鑶拌伜鍋舵毊钘佃伡鑶拌墸鑱姃鑱滄鑶拌墸鑱嵂鑱涜仭鑾借伡鍟毊鍛曟幊鑶瑰伓鑱�1闄�7
    return names


getfilelist(path, f"{projectname_old}", lambda f: os.remove(f"{path}/{f}"))
