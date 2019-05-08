#! /bin/bash
sudo apt-get update
gpg --keyserver hkp://keys.gnupg.net --recv-keys 409B6B1796C275462A1703113804BB82D39DC0E3
cd /tmp && \curl -sSL https://get.rvm.io -o rvm.sh
cat /tmp/rvm.sh | bash -s stable
source /usr/local/rvm/scripts/rvm
rvm install 2.0.0
gem install bundler
bundle install
cd ..
git clone https://github.com/cucumber/cucumber-cpp.git 
cd cucumber-cpp
git submodule init
git submodule update

cmake -E make_directory build

cmake -E chdir build cmake -DCUKE_USE_STATIC_GTEST=on -DCUKE_ENABLE_GTEST=on -DCUKE_ENABLE_EXAMPLES=on -DCMAKE_INSTALL_PREFIX=${prefix} ..

cmake --build build
cmake --build build --target test
cmake --build build --target install
