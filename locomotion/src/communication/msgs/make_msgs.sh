cd cxx

for file in ../idl/*.idl
do
    # ../../../custom/install/bin/idlc -l ../../../custom/lib/libcycloneddsidlcxx.so $file
    ../../../../custom/install/bin/idlc -l ../../../../custom/install/lib/libcycloneddsidlcxx.so $file
done
cd ../py
for file in ../idl/*.idl
do
    ../../../../custom/install/bin/idlc -l ../../../../custom/lib/libcycloneddsidlcxx.so py $file
done
cd ..
