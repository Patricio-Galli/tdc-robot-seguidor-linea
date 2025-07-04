# TODO List

## 🔧 En progreso

- [ ] Interfaz responsive

## 📌 Por hacer (Backlog)

- [X] Boton de reset
- [X] Hacer que coincidan las condiciones iniciales de kp,ki y kd con las del gráfico
- [X] Pintar línea en el gráfico de Position (+ constante LINE_WIDTH)
- [X] Hacer que el random de la perturbacion de movimiento no pueda ser menor a tanto (para que esa siempre sea grande)
- [X] Fijar los labels de los gráficos, cambian mucho de posición
- [X] Modificar cierre para elimminar logs de "invalid command"
- [X] Hacer que se vea una cantidad fija de puntos en el gráfico, no que se vaya comprimiendo
- [X] Guardar imagen del gráfico total
- [ ] Encontrar buenos valores para kp, ki, kd (Señales digitales - pixeles)
- [ ] Chequear controladores 
  - [ ] Controlador integral (errores anteriores VS mismo scan) - en este momento no hace nada
  - [ ] Chequear direcciones de los controladores, más que nada derivativo
  - [ ] ¿Proporcional on-off o por umbrales?

## 💡 Ideas futuras

- [ ] Hacer que el controlador no haga nada cuando sale de la zona de control

## Cambios

- Hacer que coincidan las condiciones iniciales de kp, ki y kd con las del gráfico
  - Las condiciones iniciales están bien, pero el gráfico al hacer reset no actualiza esos valores.



