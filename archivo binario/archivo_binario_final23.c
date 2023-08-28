#include <stdio.h>
#include <stdlib.h>

#define SIZE 24
#define CANT_TRAMAS 250 
#define repeticiones 168

int main() {

    FILE *archivo = fopen("archivo_binario.bin", "wb");
    if (archivo == NULL) {
        perror("Error al abrir el archivo");
        return 1;
    }

    unsigned char buffer = 1;
    unsigned char tram=1;
    unsigned char trama[SIZE];
     int contador;
    for (tram=1;tram<=repeticiones;tram++){
	
    	contador=0;
    	for(buffer=1;buffer<=CANT_TRAMAS;buffer++){
		
	
        trama[0] = 0x1b;
        trama[1] = buffer;
        int i;
        for ( i = 2; i <= SIZE; i++) {
            trama[i] = i + buffer;
            printf("x%dx",i);

        }



        size_t data = fwrite(trama, sizeof(unsigned char), sizeof(trama), archivo);
        if (data != sizeof(trama)) {
            perror("Error al escribir en el archivo");
            fclose(archivo);
            return 1;
        }
        contador++;
        printf("q%dq",buffer);
        printf("m%dm",tram);
	    
}
}

   
    return 0;
}

