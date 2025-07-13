% Este script carga el archivo metrics.csv y grafica las métricas de entrenamiento.
% Las columnas del CSV deben ser:
% epoch, accuracy, loss, val_accuracy, val_loss

%% Cargar datos
% Asegúrate de que el archivo 'metrics.csv' esté en la misma carpeta que este script
metrics = readtable('/MATLAB Drive/cnn_metrics/metricas.csv');

% Extraer las columnas
epochs    = metrics.epoch;
acc       = metrics.accuracy;
loss      = metrics.loss;
val_acc   = metrics.val_accuracy;
val_loss  = metrics.val_loss;

%% Gráfico combinado: Accuracy y Loss en subplots
figure('Name','Métricas de Entrenamiento y Validación','NumberTitle','off');

% Subplot 1: Accuracy
subplot(2,1,1)
plot(epochs, acc, 'bo-', 'LineWidth', 2, 'MarkerSize',6); hold on;
plot(epochs, val_acc, 'ro-', 'LineWidth', 2, 'MarkerSize',6);
xlabel('Epoch');
ylabel('Accuracy');
title('Accuracy: Entrenamiento vs. Validación');
legend('Entrenamiento', 'Validación','Location','southeast');
grid on;

% Subplot 2: Loss
subplot(2,1,2)
plot(epochs, loss, 'bo-', 'LineWidth', 2, 'MarkerSize',6); hold on;
plot(epochs, val_loss, 'ro-', 'LineWidth', 2, 'MarkerSize',6);
xlabel('Epoch');
ylabel('Loss');
title('Loss: Entrenamiento vs. Validación');
legend('Entrenamiento', 'Validación','Location','northeast');
grid on;

%% Gráficos individuales para cada métrica (opcional)
% Gráfico para Accuracy de Entrenamiento
figure('Name','Accuracy - Entrenamiento','NumberTitle','off');
plot(epochs, acc, 'b*-', 'LineWidth', 2, 'MarkerSize',6);
title('Accuracy - Entrenamiento');
xlabel('Epoch');
ylabel('Accuracy');
grid on;

% Gráfico para Accuracy de Validación
figure('Name','Accuracy - Validación','NumberTitle','off');
plot(epochs, val_acc, 'r*-', 'LineWidth', 2, 'MarkerSize',6);
title('Accuracy - Validación');
xlabel('Epoch');
ylabel('Accuracy');
grid on;

% Gráfico para Loss de Entrenamiento
figure('Name','Loss - Entrenamiento','NumberTitle','off');
plot(epochs, loss, 'b*-', 'LineWidth', 2, 'MarkerSize',6);
title('Loss - Entrenamiento');
xlabel('Epoch');
ylabel('Loss');
grid on;

% Gráfico para Loss de Validación
figure('Name','Loss - Validación','NumberTitle','off');
plot(epochs, val_loss, 'r*-', 'LineWidth', 2, 'MarkerSize',6);
title('Loss - Validación');
xlabel('Epoch');
ylabel('Loss');
grid on;

