#! /bin/bash

src_list=( $( ls src ) )
len=${#src_list[@]}

include=()
for ((i=0; i<$len; i++)); do
    include=(${include[@]} -I/src/${src_list[$i]}/include)
done

compile_flags=(g++ -Wall -std=c++17)
build_flags=(g++ -Wall)

all_obj=()
for ((i=0; i<$len; i++)); do
    search_dir=./src/${src_list[$i]}/src/
    files=( $( ls ${search_dir} ) )
    
    if [ ${#files[@]} = 0 ]; then
        continue
    fi

    for filename in $files; do
        
        fn=${filename%.*}
        ext=${filename#$fn.} 

        if [ "$ext" = "cpp" ]; then
            objf=obj/${fn}.o
            all_obj=(${all_obj[@]} $objf)
            cmd=(${compile_flags[@]} ${include[@]} -g -c $search_dir$filename -o $objf)
            echo ${cmd[@]}
            ${cmd[@]}
        fi
    done
done

files=( $( ls src/app ) )
len=${#files[@]}

for ((i=0; i<$len; i++)); do
    filename=${files[$i]}
    
    fn=${filename%.*}

    ext=${filename#$fn.} 
    if [ "$ext" = "cpp" ]; then
        objf=obj/${fn}.o   
        echo ${compile_flags[@]} src/app/$filename -o $objf
        ${compile_flags[@]} ${include[@]} -g -c src/app/$filename -o $objf
        cmd=(${build_flags[@]} ${include[@]} -o bin/${fn}.exe $objf ${all_obj[@]})
        echo ${cmd[@]}
        ${cmd[@]}
    fi
done

./bin/test.exe


