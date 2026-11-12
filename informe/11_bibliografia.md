1. Barrientos, A.; Peñín, L. F.; Balaguer, C.; Aracil, R. *Fundamentos de robótica*, 2.ª ed. McGraw‑Hill,
   2007. Usado: cap. 4 (DH, cinemática inversa, Jacobiano), cap. 5 (Lagrange, Newton–Euler, actuadores),
   cap. 6 (trayectorias) y cap. 7 (control PD, par calculado).
2. Cátedra Robótica II, FCAL‑UNER. Apuntes Temas I–VI y TP1 (robot 3: KUKA KR 6 R700). Usado: convenciones,
   Newton–Euler en {0} (Tema III §3.10), trayectorias (Tema IV), control (Tema V §§5.1–5.8), odometría y
   saturación de móviles (Tema VI).
3. Lynch, K. M.; Park, F. C. *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press,
   2017. Usado: §13.2 (robots omnidireccionales, ruedas mecanum, control) y §13.4 (odometría).
4. Spong, M. W.; Hutchinson, S.; Vidyasagar, M. *Robot Modeling and Control*. Wiley, 2006. Usado: cap. 8
   (PD con compensación de gravedad, dinámica inversa).
5. Walker, M. W.; Orin, D. E. «Efficient dynamic computer simulation of robotic mechanisms». *Journal of
   Dynamic Systems, Measurement, and Control* 104(3):205–211, 1982. Usado: obtención de M, c y g con
   Newton–Euler.
6. Ilon, B. E. *Wheels for a course stable selfpropelling vehicle movable in any desired direction on the
   ground or some other base*. Patente US 3,876,255, 1975. Usado: principio de la rueda mecanum.
7. OSOYOO. *M2.0 Metal Mecanum Robot Car Kit for Arduino Mega2560*, manual
   (m2.0-metal-mecanum-20260101.pdf), <https://osoyoo.com/manual/m2.0-metal-mecanum-20260101.pdf>, sin fecha
   de edición en la portada. Usado: medidas, conexiones driver–Mega y asignación de motores (pp. 1, 2 y 10).
8. Hoja de datos del motorreductor GM25‑370‑24140‑75‑14.5D10 (9 V, 75:1, encoder Hall de 11 PPR), según
   la recopilación de `referencias/analisis_proyecto_koba.md`. Usado: modelo del motor CC (tabla `agv.motor`).
9. Eryildiz, M. «Effect of build orientation on mechanical behaviour and build time of FDM 3D‑printed PLA
   parts: an experimental investigation». *European Mechanical Science* 5(3):116–120, 2021.
   doi:10.26701/ems.881254. Usado: resistencia del PLA según la orientación (4.B).
10. Chacón, J. M.; Caminero, M. A.; García‑Plaza, E.; Núñez, P. J. «Additive manufacturing of PLA
    structures using fused deposition modelling: effect of process parameters on mechanical properties and
    their optimal selection». *Materials & Design* 124:143–157, 2017. doi:10.1016/j.matdes.2017.03.065.
    Usado: anisotropía del PLA impreso.
11. CEN. EN 1993‑1‑8: *Eurocódigo 3, Uniones*. Usado: resistencia a corte y tracción de tornillos (4.B.5).
12. AngelLM. *Thor: open source, 3D printable robotic arm*. <https://github.com/AngelLM/Thor>. Usado:
    antecedente de brazo imprimible de 6 grados de libertad.
13. BCN3D Technologies. *BCN3D Moveo*.
    <https://www.bcn3d.com/bcn3d-moveo-the-future-of-learning-robotic-arm/>. Usado: antecedente de brazo
    educativo imprimible.
14. Coppelia Robotics. *CoppeliaSim User Manual*, versión 4.10: «Joint modes» y «ZeroMQ remote API».
    <https://manual.coppeliarobotics.com/>. Usado: modo cinemático de las juntas, construcción y verificación
    de la escena por API.
15. Microchip. *ATmega640/1280/2560 datasheet*. Usado: interrupciones por cambio de pin (PCINT16–23, puerto
    K) y temporizadores para el PWM.
16. Autodesk. *Fusion API Reference Manual* (TemporaryBRepManager, ExportManager). Usado: script del CAD.
