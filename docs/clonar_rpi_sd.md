# Clonar SD de la Rapsberry PI

Para crear una copia de seguridad de la tarjeta SD seguir estos pasos (desde Ubuntu/Linux):
## 1. Crear un fichero con la imagen completa de la Raspberry. 
Este fichero tendrá el mismo tamaño que capacidad de almacenamiento tenga la SD (aunque solo se ocupe una parte, copiará también las partes de memoria vacías, es una clonación):
```bash
sudo dd if=/dev/sda of=~/backup_raspberrypi.img bs=4M status=progress
```

``if`` es el input y ``of`` el output. Este proceso tomará su tiempo en función del tamaño máximo de la tarjeta a clonar.

Recuerda que para buscar la ruta de mount de la tarjeta SD insertada en el ordenador:
```bash
sudo fdisk -l
```

## 2. Eliminar las partes vacías de la imagen para reducir su tamaño. 
Por ejemplo, nuestra tarjeta es de 128gb pero solo están ocupados 16gb, por lo que vamos a reducir el tamaño de la imagen a esos 16gb. Utilizamos la herramienta ``PiShrink`` [https://github.com/Drewsif/PiShrink](https://github.com/Drewsif/PiShrink). Más información sobre PiShrink: [https://maskaravivek.medium.com/creating-and-shrinking-bootable-for-rasberry-pi-image-from-sd-card-d9355cb29ee8](https://maskaravivek.medium.com/creating-and-shrinking-bootable-for-rasberry-pi-image-from-sd-card-d9355cb29ee8)

Instalamos las dependencias:
```bash
sudo apt update && sudo apt install -y wget parted gzip pigz xz-utils udev e2fsprogs
```

Descargamos el ejecutable de PiShrink y le damos permisos de ejecución:
```bash
wget https://raw.githubusercontent.com/Drewsif/PiShrink/master/pishrink.sh
chmod +x pishrink.sh
sudo mv pishrink.sh /usr/local/bin
```

Ejecutar PiShrink pasando la imagen a reducir como parámetro:
```bash
sudo pishrink.sh -v backup_raspberrypi.img rpi64gb.img
```

## 3. Volcar la imagen en una tarjeta SD.

Se ha utilizado el programilla USBImager para volcar la imagen en la SD. Descargar e instalar el ``.deb``:
```bash
wget https://gitlab.com/bztsrc/usbimager/raw/binaries/usbimager_1.0.4-amd64.deb
sudo dpkg -i usbimager_1.0.4-amd64.deb
```

Abrimos el programar desde el menú de programas y mediante la ventana gráfica instalamos la imagen.

¿Otra opción (no comprobado)?: ``sudo dd if=~/rpi64gb.img of=/dev/sda bs=4M status=progress``

# Firmado: Javi :)