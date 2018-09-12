cd AirSimController 
git add -A *.cpp *.ini .gitignore `find -name CMakeLists.txt` 
cd ..
cd AX7controller 
git add -A *.cpp *.ini .gitignore `find -name CMakeLists.txt` 
cd ..
cd common
git add -A *.cpp *.hpp *.h .gitignore `find -name CMakeLists.txt`
cd ..
cd Controller
git add -A *.cpp *.hpp *.h .gitignore `find -name CMakeLists.txt`
cd ..
cd Geography
git add -A *.hpp
cd ..
cd msg
git add -A *.lcm
git add -A obu_lcm/*.hpp
cd ..
cd tools
git add -A *.cpp *.hpp *.ini *.md *.py `find -name CMakeLists.txt`
cd ..

cd Documents && git add -A . && cd ..

git add -A git_add.sh Makefile README.md
