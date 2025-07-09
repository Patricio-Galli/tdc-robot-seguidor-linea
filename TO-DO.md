# TODO List

## 🔧 En progreso

- [ ] Interfaz responsive
- [ ] Valor de referencia ajustable

## 📌 Por hacer (Backlog)

- [ ] Encontrar buenos valores para kp, ki, kd (Señales digitales - pixeles)
- [ ] Chequear controladores 
  - [ ] Controlador integral (errores anteriores VS mismo scan) - en este momento no hace nada
  - [ ] Chequear direcciones de los controladores, más que nada derivativo
  - [ ] Proporcional ON-OFF o por umbrales
- [ ] Agregar unidades y tipos de señales
- [ ] Cambiar las variables que corresponen a px (ej. feedback_signal)
- [ ] Chequear que quede actualizado el requirements.txt, readme, informe
- [ ] Chequear que no se vaya de los +-10cm el robot en position

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