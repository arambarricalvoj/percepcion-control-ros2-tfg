% Abrir y leer el fichero .csv con los datos
tabla = readtable('/MATLAB Drive/pid/performance_0205_22.5_100.0_0.0_25.0_30.csv');

% Extraer las columnas
error_pid = tabla.Var1; %.Error
tiempo = tabla.Var6; %.Timestamp

% Crear la figura y graficar
figure;
plot(tiempo, error_pid, 'b-', 'LineWidth', 2);
xlabel('Tiempo (s)', 'FontSize', 12);
ylabel('Error', 'FontSize', 12);
title('Error en función del Tiempo', 'FontSize', 14);
grid on;
