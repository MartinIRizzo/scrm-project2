#!/bin/bash

# Comprobar que se pasa un archivo como argumento
if [ $# -ne 1 ]; then
    echo "Uso: $0 archivo"
    exit 1
fi

file="$1"

# Comprobar que el archivo existe
if [ ! -f "$file" ]; then
    echo "El archivo '$file' no existe."
    exit 1
fi

sum=0
count=0

while IFS= read -r line; do
    # Comprobar que es un número
    if [[ $line =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        sum=$(echo "$sum + $line" | bc)
        ((count++))
    fi
done < "$file"

if [ $count -eq 0 ]; then
    echo "No se encontraron números válidos en el archivo."
else
    mean=$(echo "scale=2; $sum / $count" | bc)
    echo "La media es: $mean"
fi
