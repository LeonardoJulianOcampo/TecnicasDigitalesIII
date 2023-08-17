#include <stdio.h>
#include <stdlib.h>

#define SIZE 24
#define CANT_TRAMAS 5 // aprox para que el archivo pese 1mb(41667x24=1.000.000)

int main() {

    FILE *archivo = fopen("archivo_binario.bin", "wb");
    if (archivo == NULL) {
        perror("Error al abrir el archivo");
        return 1;
    }

    unsigned char contador = 1;
    unsigned char trama[SIZE];

    while(contador<=CANT_TRAMAS){
	
        trama[0] = 0x1b;
        trama[1] = contador;
        for (int i = 2; i < SIZE; i++) {
            trama[i] = i + contador;
            printf("x%dx",i);

        }



        size_t data = fwrite(trama, sizeof(unsigned char), sizeof(trama), archivo);
        if (data != sizeof(trama)) {
            perror("Error al escribir en el archivo");
            fclose(archivo);
            return 1;
        }

        contador++;
        printf("q%dq",contador);

}

   
    return 0;
}

