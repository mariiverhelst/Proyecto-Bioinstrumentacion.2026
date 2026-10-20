# Proyecto de Bioinstrumentación 2026 | Ingeniería Biomédica
## Monitor Multiparamétrico de Signos Vitales

### Descripción

Desarrollo de un monitor multiparamétrico de bajo costo basado en sistemas embebidos, orientado al monitoreo continuo de signos vitales y a la detección temprana del deterioro clínico en instituciones de atención primaria, especialmente en zonas rurales de Antioquia, Colombia.

El prototipo integra adquisición y procesamiento de señales fisiológicas, visualización local, alarmas y comunicación inalámbrica para facilitar el seguimiento del estado del paciente.

### Variables monitoreadas

ECG: adquisición de la actividad eléctrica cardíaca.

Frecuencia cardíaca (HR): cálculo de los latidos por minuto.

Fotopletismografía (PPG): adquisición de la señal óptica del pulso.

Saturación de oxígeno (SpO₂): estimación de la oxigenación sanguínea.

Temperatura corporal: medición de la temperatura del paciente.

Frecuencia respiratoria (FR): estimación de las respiraciones por minuto.

### Funciones principales

Adquisición y acondicionamiento de señales fisiológicas.

Procesamiento de datos mediante un ESP32.

Visualización local de las variables en una pantalla Nextion.

Generación de alertas visuales y sonoras según los criterios de riesgo definidos.

Transmisión inalámbrica de datos para monitorización remota.

Registro y descarga de datos fisiológicos.
