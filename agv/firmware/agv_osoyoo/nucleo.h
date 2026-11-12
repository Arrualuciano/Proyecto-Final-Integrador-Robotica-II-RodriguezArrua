// nucleo.h — Nucleo portable del AGV (sin Arduino): cinematica mecanum, odometria, PI de ruedas,
// trayectoria quintica y lazo de pose, maquina de estados y protecciones.
// Convenciones (SI): marco del cuerpo x adelante, y a la izquierda, giro + antihorario visto de arriba.
// Ruedas: 0 = FL, 1 = FR, 2 = RL, 3 = RR. Rodillos en patron X (a verificar en el kit; ver p.sy).
// w_i > 0 significa que la rueda i rueda hacia adelante (despues de aplicar signos de calibracion).
#pragma once
#include <stdint.h>

enum Modo : uint8_t { M_DESARMADO = 0, M_QUIETO = 1, M_PWM = 2, M_PASO = 3, M_VEL = 4, M_IRA = 5, M_FALLA = 6 };

// Bits de falla
#define F_COM        0x01   // watchdog de comunicacion
#define F_BAT        0x02   // bateria baja
#define F_TRABA      0x04   // rueda trabada o encoder desconectado
#define F_SOBREVEL   0x08   // sobrevelocidad (tipico de encoder con signo invertido)
#define F_PERIODO    0x10   // periodo de control excedido
#define F_PARADA     0x20   // parada de emergencia
#define F_RESET      0x40   // reinicio por watchdog de hardware detectado al arrancar

struct Parametros {
  // geometria
  float r, lx, ly;            // m
  float sy;                   // +1 patron X; -1 invierte el eje lateral si el kit resulta en patron O
  float cpr[4];               // cuentas x4 por vuelta de rueda
  uint8_t signo_motor;        // bit i: invierte el PWM de la rueda i
  uint8_t signo_enc;          // bit i: invierte el conteo de la rueda i
  // modelo identificado del accionamiento: w = K (u - udz)  (rad/s por unidad de PWM)
  float K[4], udz[4];
  // PI de rueda
  float kp, ki;               // PWM/(rad/s), PWM/rad
  float alfa_vel;             // filtro de velocidad medida: w = a w_bruta + (1 - a) w_anterior
  // lazo de pose
  float kx, ky, kphi;         // 1/s
  // limites
  float vmax, amax, wmax, alfamax;   // m/s, m/s^2, rad/s, rad/s^2 (cuerpo)
  float wrueda_max;           // rad/s, saturacion de cada rueda (escala comun)
  // calibracion de odometria (real / odometria)
  float eta_x, eta_y, eta_w;
  // seguridad y tiempos
  uint16_t ttl_ms;            // validez de una orden manual PWM / VEL
  uint16_t wd_ms;             // watchdog de comunicacion
  uint16_t traba_ms;          // tiempo con PWM alto y sin cuentas para declarar rueda trabada
  uint8_t  bat_check;         // 1 = vigilar bateria (requiere divisor instalado en A5)
  uint16_t bat_min_mV;
};

struct Estado {
  Modo modo;
  uint8_t falla;              // bits F_*
  // odometria (marco de la celda {G})
  float x, y, phi;            // m, m, rad (phi sin envolver)
  // ruedas
  float w_med[4], w_ref[4], integ[4];
  int16_t u[4];
  int32_t cnt_prev[4];
  uint8_t cnt_init;
  uint16_t traba_t[4];        // ms acumulados con PWM alto sin cuentas
  // velocidad del cuerpo comandada (despues de rampa / lazo de pose)
  float vb[3];                // vx, vy (m/s, cuerpo), wz (rad/s)
  // ordenes manuales
  int16_t u_manual[4];
  float v_manual[3];
  uint32_t t_orden_ms;        // instante de la ultima orden manual
  // ensayo de escalon
  uint8_t paso_rueda;         // 0..3, 4 = todas
  int16_t paso_u;
  uint16_t paso_ms;
  uint32_t paso_t0;
  // trayectoria quintica (IR_A)
  float tr_x0[3], tr_d[3], tr_T, tr_t;
  uint8_t tr_modo;            // 0 = abierto (solo prealimentacion), 1 = PI de ruedas, 2 = PI + lazo de pose
  float ref[3], ref_d[3];     // pose y velocidad de referencia {G}
  uint8_t tr_fin;             // 1 al terminar (se informa una vez)
  float vbat_filt;
  uint16_t bat_t;
  uint8_t pausa_com;          // 1 = corte breve de comunicacion durante un GOTO: trayectoria congelada y ruedas frenadas
};

void nucleo_parametros_defecto(Parametros &p);
void nucleo_reset(Estado &s);

// Paso de control. dt en s, cuentas crudas x4, ahora_ms, tick_pose = true cada POSE_DIV periodos.
void nucleo_paso(Estado &s, const Parametros &p, const int32_t cuentas[4], float dt,
                 uint16_t vbat_mV, uint32_t ahora_ms, bool tick_pose);

// Ordenes (devuelven 0 si se aceptan; otro valor = codigo de rechazo)
uint8_t nucleo_armar(Estado &s);
void    nucleo_desarmar(Estado &s);
void    nucleo_parar(Estado &s);                 // rampa a cero y pasa a QUIETO
void    nucleo_falla(Estado &s, uint8_t bits);
uint8_t nucleo_limpiar(Estado &s);
uint8_t nucleo_pwm(Estado &s, const int16_t u[4], uint32_t ahora_ms);
uint8_t nucleo_paso_escalon(Estado &s, uint8_t rueda, int16_t u, uint16_t ms, uint32_t ahora_ms);
uint8_t nucleo_vel(Estado &s, const Parametros &p, float vx, float vy, float wz, uint32_t ahora_ms);
uint8_t nucleo_ir_a(Estado &s, const Parametros &p, float x, float y, float phi, float T, uint8_t modo, float *T_efectivo);
void    nucleo_set_pose(Estado &s, float x, float y, float phi);

// Cinematica (publica para pruebas)
void cin_inversa(const Parametros &p, const float vb[3], float w[4]);    // cuerpo -> ruedas (rad/s)
void cin_directa(const Parametros &p, const float w[4], float vb[3]);    // ruedas -> cuerpo (minimos cuadrados, H+)
float envolver(float a);                                                 // a (-pi, pi]
