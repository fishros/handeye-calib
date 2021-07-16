sudo cp librsaubo.so.* /usr/local/lib;
sudo cp liblog4cplus-1.2.so.* /usr/local/lib;
sudo cp libconfig.so.* /usr/local/lib;
sudo cp libprotobuf.so.* /usr/local/lib/;
sudo rm /usr/local/lib/librsaubo.so.1 /usr/local/lib/libconfig.so.11 
sudo sh -c 'echo "include /usr/local/lib" >> /etc/ld.so.conf'
sudo ldconfig
