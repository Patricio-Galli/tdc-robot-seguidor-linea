# TODO List

## 🔧 En progreso

- [ ] Interfaz responsive

## 📌 Por hacer (Backlog)

- [ ] Chequear controladores 
  - [ ] Controlador integral (errores anteriores VS mismo scan)
  - [ ] Chequear direcciones de los controladores, más que nada derivativo
  - [ ] Proporcional ON-OFF o por umbrales

## 💡 Ideas futuras

- [ ] Hacer que el controlador no haga nada cuando sale de la zona de control
- [ ] Agregar diagrama de bloques para ejemplificar cada gráfico
- [ ] Hacer que el gráfico muestre una señal discreta cuando corresponde

## Hecho

- [X] Boton de reset
  - [X] Hacer que coincidan las condiciones iniciales de kp, ki y kd con las del gráfico
- [X] Pintar línea en el gráfico de Position (LINE_WIDTH)
- [X] Hacer que el random de la perturbacion de movimiento no pueda ser tan pequeño (limitar valor inferior)
- [X] Fijar los labels de los gráficos, cambian mucho de posición
- [X] Modificar cierre para elimminar logs de "invalid command"
- [X] Hacer que se vea una cantidad fija de puntos en el gráfico, no que se vaya comprimiendo
- [X] Guardar imagen del gráfico total
- [X] Botón de pausa
- [X] Agregar gráficos de valor de referencia y señal medida
- [X] Hacer que se vea la línea de posición por sobre la de velocidad
- [X] Permitir ocultar y mostrar cada gráfico
- [X] Sacar etiquetas redundantes
- [X] Valor de referencia ajustable
- [X] Agregar unidades y tipos de señales
- [X] Cambiar las variables que corresponen a px (ej. feedback_signal)
- [X] Mejorar yticks de px (números redondos)