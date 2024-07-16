cd build
make
cd ..
dir="swp"
for file in `ls $dir`
do
	build/a1 swp/$file
done

