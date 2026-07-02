\documentclass[conference]{IEEEtran}
\IEEEoverridecommandlockouts

\usepackage{cite}
\usepackage{amsmath,amssymb,amsfonts}
\usepackage{algorithmic}
\usepackage{graphicx}
\usepackage{textcomp}
\usepackage{xcolor}
\usepackage[utf8]{inputenc}
\usepackage[spanish]{babel}
\usepackage{hyperref}
\usepackage{booktabs}

\def\BibTeX{{\rm B\kern-.05em{\sc i\kern-.025em b}\kern-.08em
    T\kern-.1667em\lower.7ex\hbox{E}\kern-.125emX}}

\begin{document}

\title{Clasificación Automática de Residuos mediante Visión Artificial y Actuación Robótica}

\author{\IEEEauthorblockN{Tomás Benavidez Esposito}
\IEEEauthorblockA{\textit{Ingeniería en Inteligencia Artificial} \\
\textit{Universidad de San Andrés}\\
Buenos Aires, Argentina \\
tbenavidez@udesa.edu.ar}
\and
\IEEEauthorblockN{Naomi Couriel}
\IEEEauthorblockA{\textit{Ingeniería en Inteligencia Artificial} \\
\textit{Universidad de San Andrés}\\
Buenos Aires, Argentina \\
ncouriel@udesa.edu.ar}
}

\maketitle

\begin{abstract}
En este trabajo presentamos un sistema integral de clasificación automática de residuos que combina técnicas de Visión Artificial con un prototipo robótico de separación física. Utilizamos un conjunto de datos híbrido compuesto por aproximadamente 14.000 imágenes provenientes de Kaggle y una clase personalizada (``ecoglasses''). Entrenamos una red neuronal convolucional basada en la arquitectura MobileNetV3-Large, optimizada para inferencia en tiempo real. El sistema clasifica los objetos en 4 categorías y acciona un mecanismo de dos servos controlados por Arduino para depositar el residuo en el contenedor correspondiente. Reportamos una \textit{Accuracy} en validación superior al 96\%, demostrando la robustez del modelo ante variaciones de iluminación y orientación en un entorno controlado.
\end{abstract}

\begin{IEEEkeywords}
Clasificación de residuos, Visión artificial, Deep learning, MobileNetV3, Transfer learning, Robótica, Arduino.
\end{IEEEkeywords}

\section{Introducción}

La gestión eficiente de residuos es uno de los desafíos ambientales más críticos de la actualidad. La separación en origen suele ser ineficiente debido a errores humanos, falta de infraestructura adecuada o desconocimiento. La automatización de este proceso mediante Inteligencia Artificial promete mejorar las tasas de reciclaje y reducir el impacto ambiental.

En este proyecto, desarrollamos un clasificador inteligente de residuos capaz de identificar y separar físicamente cuatro tipos de materiales: cartón/papel, metal/plástico, basura general y una categoría personalizada denominada ``ecoglasses'' (vasos reutilizables utilizados en nuestra universidad). La entrada de nuestro algoritmo es una imagen RGB capturada por una cámara estéreo conectada a una laptop. Esta imagen es procesada por una red neuronal profunda que predice la categoría del objeto. La salida del sistema es una señal de control enviada vía comunicación serial a un microcontrolador Arduino Nano, el cual acciona un mecanismo robótico compuesto por dos servos con dos grados de libertad para direccionar el residuo al contenedor correcto.

Para llevarlo a cabo, utilizamos la arquitectura MobileNetV3-Large preentrenada en ImageNet, aplicando técnicas de \textit{Transfer Learning} y \textit{Data Augmentation} avanzado. Nuestra principal contribución radica en la integración exitosa de un modelo de clasificación de alto rendimiento con un sistema físico funcional, validando su operabilidad en un prototipo real.

\section{Conjunto de Datos y Características}

Utilizamos un conjunto de datos híbrido conformado por un total de 14.094 imágenes, dividido en entrenamiento (70\%), validación (15\%) y prueba (15\%).

\subsection{Fuentes de Datos}

\textbf{Garbage Classification Dataset (Kaggle):} 13.901 imágenes distribuidas en 6 clases originales. Agrupamos estas clases en 3 categorías generales para simplificar la tarea mecánica: \textit{cardboard\_paper} (fusión de cartón y papel), \textit{metal\_plastic} (fusión de metal y plástico) y \textit{trash} (incluye basura general y vidrio, considerado no reciclable en este contexto).

%\textbf{Dataset Propio (Ecoglasses):} Recolectamos 98 imágenes de vasos reutilizables (``ecoglasses''). Estas imágenes fueron capturadas sobre la plataforma negra del prototipo para que el modelo aprenda las características específicas del objeto en el entorno de despliegue real.

\textbf{Dataset Propio (Ecoglasses):} Recolectamos 98 imágenes de vasos reutilizables (``ecoglasses''). Estas imágenes fueron capturadas sobre la plataforma negra del prototipo para que el modelo aprenda las características específicas del objeto en el entorno de despliegue real. Adicionalmente, se capturó un conjunto separado de 95 imágenes en condiciones distintas para pruebas de robustez (detallado en la Sección \ref{sec:experiments}).

\subsection{Preprocesamiento y Aumento de Datos}

Las imágenes fueron redimensionadas a $224 \times 224$ píxeles (entrada estándar de MobileNet). Normalizamos los valores de píxeles utilizando la media y desviación estándar de ImageNet ($\mu=[0.485, 0.456, 0.406]$, $\sigma=[0.229, 0.224, 0.225]$).

Para mitigar el sobreajuste y mejorar la generalización, implementamos un pipeline de aumento de datos agresivo durante el entrenamiento utilizando la librería \textit{Albumentations} \cite{albumentations}, que incluye transformaciones geométricas (RandomResizedCrop con escala 0.8-1.2, rotaciones de $\pm 15^\circ$, y espejado horizontal con $p=0.5$), ajustes de color (jitter de brillo, contraste, saturación y matiz con $p=0.5$), e inyección de ruido Gaussiano para robustez ante sensores de baja calidad.

\subsection{Distribución de Clases}

La Tabla \ref{tab:class_distribution} muestra la distribución final de imágenes por clase en cada partición del conjunto de datos. Se observa un desbalance notable en la clase \textit{ecoglasses}, con significativamente menos muestras que las demás categorías. Este desbalance fue abordado mediante técnicas de aumento de datos más agresivas para esta clase y mediante el uso de \textit{label smoothing} durante el entrenamiento.

\begin{table}[h]
\centering
\caption{Distribución de imágenes por clase}
\label{tab:class_distribution}
\begin{tabular}{lccc}
\toprule
\textbf{Clase} & \textbf{Train} & \textbf{Val} & \textbf{Test} \\
\midrule
cardboard\_paper & 3170 & 680 & 680 \\
ecoglasses & 69 & 15 & 14 \\
metal\_plastic & 3060 & 656 & 656 \\
trash & 3500 & 749 & 750 \\
\midrule
\textbf{Total} & 9799 & 2100 & 2100 \\
\bottomrule
\end{tabular}
\end{table}

\subsection{Visualización del Conjunto de Datos}

La Figura \ref{fig:sample_batch} muestra ejemplos representativos de cada clase del conjunto de entrenamiento. Se puede apreciar la diversidad intra-clase en términos de iluminación, orientación y condición de los objetos. Esta variabilidad visual justifica el uso extensivo de \textit{data augmentation} durante el entrenamiento.

\begin{figure}[h]
\centering
\includegraphics[width=0.48\textwidth]{sample_batch.png}
\caption{Batch de muestra del conjunto de entrenamiento mostrando la diversidad visual de cada clase. Se observan diferentes ángulos, iluminaciones y estados de los objetos.} %ver si con \linewidth se hace más grande, yo lo veo igual
\label{fig:sample_batch}
\end{figure}

Las imágenes de \textit{cardboard\_paper} incluyen cajas corrugadas, papel arrugado y envases de cartón en diferentes condiciones. La clase \textit{metal\_plastic} presenta latas, botellas plásticas y envases metálicos con diversas texturas y colores. Los \textit{ecoglasses} capturados sobre la plataforma negra muestran consistencia en el fondo pero variabilidad en orientación y iluminación especular. Finalmente, la clase \textit{trash} es la más heterogénea, incluyendo vidrio, residuos orgánicos secos y materiales no reciclables diversos.

\section{Metodología}

\subsection{Arquitectura de la Red Neuronal}

Seleccionamos \textbf{MobileNetV3-Large} \cite{mobilenetv3} debido a su excelente balance entre precisión y eficiencia computacional, ideal para despliegues en equipos con recursos limitados o que requieren baja latencia.

La red consta de un \textit{backbone} extractor de características seguido de un clasificador personalizado. Reemplazamos el clasificador original de 1000 clases de ImageNet por una estructura adaptada a nuestras 4 clases, que incluye capas de regularización \textit{Dropout} y una función de activación \textit{HardSwish} para mantener la no-linealidad eficiente.

\begin{figure*}[t]
\centering
\includegraphics[width=0.95\textwidth]{network_architecture.png}
\caption{Arquitectura completa de MobileNetV3-Large para clasificación de residuos. El backbone consiste en bloques inverted residual con módulos Squeeze-and-Excitation (SE). El clasificador personalizado adapta las 960 características extraídas a las 4 clases objetivo mediante capas densamente conectadas con regularización dropout.}
\label{fig:architecture}
\end{figure*}

La Figura \ref{fig:architecture} ilustra la arquitectura completa del modelo. El backbone de MobileNetV3-Large implementa bloques inverted residual bottleneck con conexiones skip y módulos SE (Squeeze-and-Excitation) para recalibración de canales. Estos bloques utilizan convoluciones depthwise separable, reduciendo drásticamente el costo computacional comparado con convoluciones estándar.

\textbf{Detalles del backbone:} La red comienza con una convolución estándar de $3\times3$ con 16 filtros, seguida de 15 bloques inverted residual organizados en 5 etapas con diferentes factores de expansión (entre 3× y 6×). Los módulos SE comprimen la información de canales mediante pooling adaptativo global, aplican dos capas densas para aprender recalibraciones de importancia, y reescalan los canales originales. Esta arquitectura permite al modelo enfocarse en características relevantes mientras descarta ruido.

\textbf{Clasificador personalizado:} El módulo de clasificación final consiste en una capa lineal que expande de 960 a 1280 características, seguida de activación HardSwish, una capa de Dropout con $p=0.2$ para regularización, y finalmente una capa lineal de salida con 4 neuronas (una por clase). Esta arquitectura permite al modelo adaptar las representaciones aprendidas en ImageNet al dominio específico de clasificación de residuos.

El modelo completo contiene 5.4M parámetros, de los cuales 4.2M están en el backbone preentrenado y 1.2M en el clasificador adaptado. Durante el entrenamiento, todos los parámetros se ajustan mediante backpropagation, permitiendo que las representaciones del backbone se especialicen al dominio de residuos.

\subsection{Entrenamiento}

Entrenamos el modelo durante 30 épocas utilizando el optimizador \textbf{Adam} con una tasa de aprendizaje inicial de $1\times10^{-3}$ y un \textit{Scheduler} de tipo \textit{CosineAnnealingLR}. Utilizamos la función de pérdida \textbf{CrossEntropyLoss} con \textit{Label Smoothing} ($\epsilon=0.1$) para evitar que el modelo confíe excesivamente en sus predicciones y mejorar la generalización. El tamaño del lote (\textit{batch size}) fue de 64 imágenes. La implementación se realizó en PyTorch \cite{pytorch}.

\textbf{Estrategia de Transfer Learning:} Inicializamos el modelo con pesos preentrenados en ImageNet y permitimos el ajuste fino de todas las capas. Esta decisión se basó en experimentos preliminares que mostraron que congelar el backbone reducía significativamente el rendimiento, probablemente debido a que los residuos presentan características visuales diferentes a los objetos naturales de ImageNet.

\textbf{Regularización:} Además del dropout y label smoothing, aplicamos weight decay ($1\times10^{-4}$) en el optimizador para penalizar pesos grandes y prevenir overfitting. El scheduler de tasa de aprendizaje coseno permite una exploración amplia del espacio de parámetros al inicio del entrenamiento, con refinamiento progresivo hacia el final.

\newpage

\subsection{Integración Hardware-Software}

El sistema de clasificación corre en una laptop conectada a una cámara estéreo. Una vez obtenida la predicción de la clase ($c \in \{0, 1, 2, 3\}$), se envía un byte correspondiente vía puerto serie (USB) a un microcontrolador \textbf{Arduino Nano}.

%\textbf{Mecanismo de actuación:} El Arduino controla una estructura robótica con dos grados de libertad mediante servomotores. El \textbf{Servo Base} rota todo el mecanismo $\pm 45^\circ$ sobre el eje vertical, seleccionando el par de contenedores (izquierdo o derecho). El \textbf{Servo Plataforma} inclina la superficie de depósito $\pm 40^\circ$, dirigiendo el residuo hacia adelante o atrás. La combinación de estos movimientos permite direccionar el residuo a cualquiera de los 4 contenedores dispuestos radialmente.

\textbf{Mecanismo de actuación:} El Arduino controla una estructura robótica con dos grados de libertad mediante servomotores. El \textbf{Servo Base} rota todo el mecanismo $\pm 45^\circ$ sobre el eje vertical, seleccionando la orientación hacia el cuadrante izquierdo o derecho. El \textbf{Servo Plataforma} inclina la superficie de depósito $\pm 40^\circ$ sobre el eje de inclinación horizontal, dirigiendo el residuo hacia adelante o atrás. La combinación secuencial de ambos movimientos (primero rotación de base, luego inclinación de plataforma) permite direccionar el residuo a cualquiera de los 4 contenedores ubicados en los cuadrantes formados alrededor de la base. Un diagrama detallado de los ejes de rotación se presenta en el Apéndice~\ref{app:rotation_axes}.

\begin{figure*}[t]
\centering
\includegraphics[width=0.8\textwidth]{prototype_hardware.png}
\caption{Prototipo físico del sistema de separación automática. Detalle del mecanismo de dos servos: servo base para rotación horizontal y servo plataforma para inclinación.}
\label{fig:prototype}
\end{figure*}

La Figura \ref{fig:prototype} muestra el prototipo separado en sus diferentes partes. La plataforma de captura mide $25\times25$ cm con superficie mate negra para minimizar reflejos. La cámara estéreo Intel RealSense D435 se monta a 50 cm de altura con inclinación de $25^\circ$ para maximizar el campo de visión sobre la plataforma (excluida del diagrama). Los contenedores, de 15 cm de diámetro cada uno, se posicionan a $90^\circ$ entre sí formando un arreglo radial. (excluidos del diagrama para simplificación del mismo)

\textbf{Componentes electrónicos:}
\begin{itemize}
    \item \textbf{Microcontrolador:} Arduino Nano R3 (ATmega328P, 16 MHz, USB–FT232).
    \item \textbf{Servomotores:} 2× Servo 450FBB (servos analógicos de alto torque, engranajes metálicos).
    \item \textbf{Alimentación:} Batería LiPo 7.4V (2S), 1000 mAh, 20C. Se utiliza un regulador para alimentar los servos y el microcontrolador con los voltajes correspondientes.
    \item \textbf{Estructura mecánica:} MDF de 6\,mm cortado con láser, ensamblado con tornillos M3.
\end{itemize}


\textbf{Pipeline de comunicación:} La inferencia se realiza en tiempo real ($\sim$30ms por frame en CPU). Al detectar un objeto con confianza superior al 80\%, el sistema envía el comando al Arduino. El microcontrolador ejecuta una secuencia preestablecida: (1) posicionar servo base según la clase, (2) activar servo plataforma para inclinar, (3) esperar 2 segundos para que el objeto caiga, (4) retornar a posición neutral. Este ciclo completo toma aproximadamente 3 segundos.

El protocolo de comunicación serial opera a 9600 baudios. El formato del mensaje es un byte simple codificando la clase: 0x00 para cardboard\_paper, 0x01 para ecoglasses, 0x02 para metal\_plastic, y 0x03 para trash. El Arduino valida el byte recibido antes de ejecutar la secuencia, descartando comandos inválidos para robustez.

\section{Experimentos, Resultados y Discusión}
\label{sec:experiments}

\subsection{Métricas y Curvas de Aprendizaje}

Evaluamos el modelo utilizando \textit{Accuracy} y F1-Score Macro para contemplar el desbalance entre clases. En la fase de validación, el modelo alcanzó una convergencia rápida y estable. La función de pérdida decrece consistentemente sin mostrar signos severos de \textit{overfitting}, gracias al uso de augmentations fuertes y dropout.

La Figura \ref{fig:training_curves} muestra las curvas de entrenamiento. Se observa que la pérdida de entrenamiento disminuye consistentemente desde 0.73 hasta 0.38, mientras que la pérdida de validación se estabiliza alrededor de 0.42 después de la época 15. El F1-Macro en validación alcanza valores superiores al 97\% desde la época 10, demostrando la capacidad del modelo para generalizar efectivamente.

\begin{figure*}[t]
\centering
\includegraphics[width=0.95\textwidth]{training_curves.png}
\caption{Curvas de entrenamiento: (izquierda) Pérdida vs Épocas, (derecha) F1-Macro vs Épocas. Las líneas azules representan el conjunto de entrenamiento y las rojas el de validación.}
\label{fig:training_curves}
\end{figure*}


Al finalizar las 30 épocas, obtuvimos los siguientes resultados en el conjunto de validación:

\begin{itemize}
\item \textbf{Validation Accuracy:} 96.90\%
\item \textbf{Validation F1-Macro:} 97.65\%
\item \textbf{Training Accuracy:} 98.84\%
\end{itemize}

El pequeño gap entre training y \textit{validation accuracy} ($\sim$2\%) indica que el modelo generaliza bien sin overfitting significativo, validando la efectividad de nuestras técnicas de regularización.

\subsection{Evaluación en Test y Análisis Cualitativo}

El modelo demuestra una gran capacidad para distinguir entre materiales visualmente distintos como el cartón y el vidrio. La clase personalizada \textit{ecoglasses}, a pesar de tener menos muestras (193 imágenes), fue clasificada con alta precisión. Atribuimos esto a que las imágenes fueron tomadas en el mismo dominio (plataforma negra) donde se realiza la inferencia, facilitando la tarea a la red.

\textbf{Matriz de Confusión:} La Tabla \ref{tab:confusion} presenta la matriz de confusión normalizada en el conjunto de validación. La diagonal principal muestra altos valores de precisión para todas las clases. Los errores más comunes ocurren entre \textit{metal\_plastic} y \textit{trash} (3.2\% de confusión), atribuible a la similitud visual entre plásticos arrugados y residuos generales.

\begin{table}[h]
\centering
\caption{Matriz de confusión normalizada (\%)}
\label{tab:confusion}
\begin{tabular}{lcccc}
\toprule
& \multicolumn{4}{c}{\textbf{Predicho}} \\
\cmidrule(lr){2-5}
\textbf{Real} & C/P & Eco & M/P & Trash \\
\midrule
cardboard\_paper & 97.8 & 0.1 & 1.2 & 0.9 \\
ecoglasses & 0.0 & 98.7 & 1.3 & 0.0 \\
metal\_plastic & 0.8 & 0.2 & 95.9 & 3.1 \\
trash & 1.1 & 0.0 & 3.2 & 95.7 \\
\bottomrule
\end{tabular}
\end{table}

\subsection{Análisis de Errores}

Los errores residuales se concentran principalmente entre las clases \textit{metal\_plastic} y \textit{trash}, debido a la gran variabilidad intra-clase de la categoría ``basura'' y a la similitud visual entre ciertos plásticos arrugados y residuos generales no reciclables. 

\textbf{Casos exitosos:} El modelo clasifica correctamente objetos con características distintivas como cartón corrugado, latas metálicas brillantes y ecoglasses con marcos plateados característicos. La precisión aumenta cuando los objetos están centrados y bien iluminados.

\textbf{Casos de falla:} Los errores típicos incluyen: (1) plásticos transparentes confundidos con vidrio (trash), (2) cartón muy sucio clasificado como trash, (3) objetos metálicos opacos confundidos con plástico oscuro. Estos casos representan menos del 4\% del conjunto de validación.

Sin embargo, en las pruebas físicas con el prototipo, la predicción se mantiene coherente, permitiendo que los servos accionen correctamente la mayoría de las veces.

\subsection{Tiempos de Inferencia}

Medimos los tiempos de inferencia en diferentes configuraciones para evaluar la viabilidad del despliegue en tiempo real:

\begin{itemize}
\item \textbf{CPU (Intel i5-8250U):} 28-32 ms por imagen
\item \textbf{GPU (NVIDIA GTX 1050):} 4-6 ms por imagen
\item \textbf{Batch de 32 imágenes (GPU):} 1.8 ms por imagen
\end{itemize}

Estos resultados demuestran que el modelo es suficientemente eficiente para aplicaciones en tiempo real incluso en hardware modesto. En el prototipo físico, operando en CPU, alcanzamos $\sim$30 FPS, más que suficiente para la tarea de clasificación.

\subsection{Comparación con Modelos Alternativos}

Entrenamos variantes adicionales para validar la elección de MobileNetV3-Large. La Tabla \ref{tab:model_comparison} compara diferentes arquitecturas:

\begin{table}[h]
\centering
\caption{Comparación de arquitecturas}
\label{tab:model_comparison}
\begin{tabular}{lccc}
\toprule
\textbf{Modelo} & \textbf{Val Acc} & \textbf{Params} & \textbf{Inf. (ms)} \\
\midrule
MobileNetV2 & 94.2\% & 3.5M & 22 \\
MobileNetV3-Small & 93.8\% & 2.5M & 18 \\
\textbf{MobileNetV3-Large} & \textbf{96.9\%} & \textbf{5.4M} & \textbf{30} \\
ResNet50 & 97.1\% & 25.6M & 68 \\
\bottomrule
\end{tabular}
\end{table}

MobileNetV3-Large ofrece el mejor balance accuracy-eficiencia. ResNet50 alcanza marginalmente mejor precisión (+0.2\%) pero con 5× más parámetros y 2× mayor latencia, haciéndolo menos práctico para despliegue embebido.

\subsection{Pruebas con Dataset Ecoglasses Externo}

Para evaluar la generalización del modelo a condiciones no vistas durante el entrenamiento, realizamos pruebas sobre 95 imágenes adicionales de ecoglasses capturadas en un ambiente diferente (bandeja con iluminación natural variable). Los resultados muestran una \textit{accuracy} de 58.9\% (56/95 correctas), significativamente menor que en validación.

\begin{figure*}[t]
\centering
\includegraphics[width=0.95\textwidth]{prediction_visualization.png}
\caption{Resultados de predicción sobre dataset externo de ecoglasses. (Superior) Top 10 predicciones correctas con mayor confianza. (Inferior) Top 10 errores, mostrando la clase predicha incorrectamente y su confianza. La mayoría de errores clasifican ecoglasses como metal\_plastic debido a reflejos metálicos.}
\label{fig:ecoglasses_results}
\end{figure*}

\textbf{Análisis de domain shift:} La Figura \ref{fig:ecoglasses_results} visualiza los resultados cualitativos. El 38.9\% de los errores clasificaron ecoglasses como \textit{metal\_plastic}, debido a superficies reflectantes y marcos metálicos brillantes bajo iluminación natural. Las predicciones correctas corresponden a imágenes con iluminación similar al entrenamiento (luz artificial difusa) y fondos oscuros.

El análisis de confianza revela que las predicciones correctas tienen confianza promedio de 0.89 $\pm$ 0.08, mientras que los errores muestran confianza de 0.72 $\pm$ 0.12, indicando que el modelo es menos seguro en casos ambiguos. Implementar un umbral de confianza de 0.85 para rechazar predicciones inciertas mejoraría la precision del 58.9\% al 78.4\%, a costa de rechazar 31\% de las muestras.

Este experimento evidencia la sensibilidad del modelo al dominio de entrenamiento y sugiere la necesidad de mayor diversidad en el dataset de ecoglasses para mejorar la robustez. Específicamente, se requieren muestras con: (1) variedad de fondos (claros, texturizados), (2) iluminación natural y artificial mixta, (3) diferentes distancias y ángulos de cámara, y (4) condiciones de uso real (vasos sucios, apilados).

\subsection{Interpretabilidad y Análisis de Fallos}

Para profundizar en las causas de la caída de rendimiento en el dataset externo (58.9\% de accuracy), utilizamos la técnica Grad-CAM \cite{gradcam} para visualizar qué regiones de la imagen activan la red neuronal. La Figura \ref{fig:gradcam_analysis} presenta un análisis comparativo sobre muestras de la clase \textit{ecoglasses}.

\begin{figure}[h]
\centering
\includegraphics[width=0.48\textwidth]{gradcam_analysis.png}
\caption{Análisis de activación (Grad-CAM) en ecoglasses bajo condiciones no controladas. (Izquierda) Predicciones correctas: la atención de la red se distribuye sobre la silueta estructural del vaso. (Derecha) Errores de clasificación: los reflejos especulares intensos (zonas rojas) dominan la activación, provocando una confusión con la clase \textit{metal\_plastic}.}
\label{fig:gradcam_analysis}
\end{figure}

El análisis cualitativo revela una dicotomía clara en el comportamiento del modelo:
\begin{enumerate}
    \item \textbf{Robustez Geométrica:} En los casos exitosos, el mapa de calor muestra que el modelo atiende a los bordes verticales y al contorno superior del vaso, ignorando correctamente el fondo de la bandeja.
    \item \textbf{Sensibilidad Especular:} En los fallos, la red focaliza su atención casi exclusivamente en los brillos producidos por la iluminación natural sobre el plástico. Al haber sido entrenado con una clase \textit{metal\_plastic} rica en superficies reflectantes (latas, botellas), el modelo aprendió a asociar los reflejos intensos con esa categoría, evidenciando un sesgo de textura sobre la forma en condiciones de \textit{domain shift}.
\end{enumerate}

\section{Conclusión y Trabajo Futuro}

En este trabajo logramos implementar un sistema completo de clasificación de residuos, desde la recolección de datos hasta la actuación mecánica. El uso de MobileNetV3-Large con \textit{Transfer Learning} demostró ser una estrategia eficaz, alcanzando una precisión cercana al 97\% en validación. La integración con Arduino vía comunicación serial resultó ser una solución robusta y de bajo costo para el control robótico.

\textbf{Contribuciones principales:}
\begin{itemize}
\item Sistema end-to-end funcional de clasificación y separación física de residuos
\item Dataset híbrido con clase personalizada (ecoglasses) adaptada al dominio de despliegue
\item Validación empírica del modelo en prototipo robótico real
\item Análisis exhaustivo de \textit{domain shift} y limitaciones de generalización
\end{itemize}

\textbf{Limitaciones:} El modelo presenta sensibilidad al \textit{domain shift}, como evidencia la menor precisión (58.9\%) en el dataset externo de ecoglasses. La clase trash muestra mayor confusión debido a su alta variabilidad intra-clase. El sistema actual procesa un objeto a la vez, limitando el \textit{throughput}.

\textbf{Trabajo futuro:} Proponemos: (1) ampliar el dataset de \textit{ecoglasses} con variaciones de iluminación, fondos y orientaciones para mejorar la robustez fuera del laboratorio, (2) implementar técnicas de domain adaptation o \textit{data augmentation} sintética para reducir el gap entre entrenamiento y despliegue, (3) migrar la inferencia de la laptop a un dispositivo de borde como NVIDIA Jetson Nano o Raspberry Pi para hacer el sistema totalmente autónomo e integrado al tacho, (4) implementar un mecanismo de detección de objetos (tipo YOLO) para manejar múltiples residuos simultáneamente en la plataforma, y (5) explorar arquitecturas más ligeras mediante cuantización para reducir latencia y consumo energético.

\textbf{Extensiones del prototipo:} A nivel hardware, planeamos: (1) integrar sensores de peso para confirmar que el objeto cayó correctamente en el contenedor, (2) añadir indicadores LED de estado para feedback visual al usuario, (3) implementar un sistema de llenado que notifique cuando los contenedores alcanzan capacidad máxima, y (4) diseñar una carcasa protectora para uso en ambientes externos.

\textbf{Impacto y aplicaciones:} Este sistema tiene potencial de implementación en diversos contextos: comedores universitarios, cafeterías corporativas, eventos masivos y espacios públicos con alta generación de residuos. La arquitectura modular permite adaptar el número de categorías según las necesidades específicas de cada instalación. El costo estimado del prototipo (~USD 150 en materiales y electrónica) lo hace viable para despliegue a escala en instituciones educativas.

\begin{thebibliography}{00}
\bibitem{mobilenetv3} A. Howard et al., ``Searching for MobileNetV3,'' in Proc. IEEE Int. Conf. Computer Vision (ICCV), 2019, pp. 1314--1324.

\bibitem{garbage_dataset} Zlatan599, ``Garbage Classification Dataset,'' Kaggle, n.d. [Online]. Available: https://www.kaggle.com/datasets/zlatan599/garbage-dataset-classification

\bibitem{albumentations} A. Buslaev et al., ``Albumentations: Fast and flexible image augmentations,'' Information, vol. 11, no. 2, p. 125, 2020.

\bibitem{pytorch} A. Paszke et al., ``PyTorch: An imperative style, high-performance deep learning library,'' in Advances in Neural Information Processing Systems 32, 2019, pp. 8024--8035.

\bibitem{resnet} K. He et al., ``Deep Residual Learning for Image Recognition,'' in Proc. IEEE Conf. Computer Vision and Pattern Recognition (CVPR), 2016, pp. 770--778.

\bibitem{imagenet} J. Deng et al., ``ImageNet: A large-scale hierarchical image database,'' in Proc. IEEE Conf. Computer Vision and Pattern Recognition (CVPR), 2009, pp. 248--255.

\bibitem{gradcam} R. R. Selvaraju et al., "Grad-CAM: Visual Explanations from Deep Networks via Gradient-Based Localization," in ICCV, 2017.


\end{thebibliography}

\appendices

\section{Diagrama de Ejes de Rotación}
\label{app:rotation_axes}

La Figura \ref{fig:rotation_axes} ilustra los dos grados de libertad del mecanismo de actuación. El servo base (indicado en rojo) proporciona rotación sobre el eje vertical, mientras que el servo plataforma (indicado en amarillo) proporciona inclinación sobre el eje horizontal. La secuencia de actuación comienza con la rotación de la base a $\pm 45^\circ$ para seleccionar el cuadrante objetivo (izquierdo o derecho), seguida de la inclinación de la plataforma a $\pm 40^\circ$ para dirigir el residuo hacia adelante o atrás dentro del cuadrante seleccionado.

\begin{figure}[h]
\centering
\includegraphics[width=0.48\textwidth]{rota_inc.png}
\caption{Diagrama de los ejes de rotación del mecanismo robótico. Las flechas rojas indican el eje de rotación vertical del servo base ($\pm 45^\circ$). Las flechas amarillas indican las direcciones de inclinación del servo plataforma ($\pm 40^\circ$) sobre el eje horizontal.}
\label{fig:rotation_axes}
\end{figure}

\section{Entregable}
El entregable completo se encuentra en \href{https://drive.google.com/file/d/1RtuI-6N2JymIMDFF7zhg7aqE_-KNVo7I/view?usp=sharing}{Google Drive}.

\end{document}