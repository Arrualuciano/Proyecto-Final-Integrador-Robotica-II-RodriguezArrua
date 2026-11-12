-- reproduccion.lua — Script de simulación de la escena celda_integrador.ttt (CoppeliaSim 4.10).
-- Reproduce el movimiento CALCULADO en Octave (no lo genera CoppeliaSim):
--   * juntas J1..J6 del brazo en MODO CINEMÁTICO: sim.setJointPosition con q real del lazo de control;
--   * AGV y pieza como objetos estáticos movidos con sim.setObjectPose.
-- No se usa el motor de física ni los controladores internos de las juntas (todas las formas son
-- estáticas y no respondientes). Datos: reproduccion.csv en la carpeta de la escena (50 Hz, interpolado).
sim = require 'sim'

local datos, n, dt, hJ, hAGV, hPieza

local function leerCSV(ruta)
    local f = io.open(ruta, 'r')
    if not f then error('No se encontró ' .. ruta .. ' (correr coppelia/preparar_reproduccion.py)') end
    f:read('*l')                                     -- encabezado
    local filas = {}
    for linea in f:lines() do
        local fila = {}
        for v in string.gmatch(linea, '[^,]+') do fila[#fila + 1] = tonumber(v) end
        if #fila >= 18 then filas[#filas + 1] = fila end
    end
    f:close()
    return filas
end

function sysCall_init()
    datos = leerCSV(sim.getStringParam(sim.stringparam_scene_path) .. '/reproduccion.csv')
    n = #datos
    dt = datos[2][1] - datos[1][1]
    local porNombre = {}
    for _, h in ipairs(sim.getObjectsInTree(sim.handle_scene)) do porNombre[sim.getObjectAlias(h)] = h end
    hJ = {}
    for i = 1, 6 do hJ[i] = porNombre['J' .. i] end
    hAGV = porNombre['AGV']
    hPieza = porNombre['Pieza']
    print(string.format('Reproducción: %d muestras, %.1f s', n, datos[n][1]))
end

function sysCall_actuation()
    local t = sim.getSimulationTime()
    local k = math.floor(t / dt) + 1
    if k > n - 1 then k = n - 1 end
    local a = math.min(math.max((t - datos[k][1]) / dt, 0), 1)
    local function v(c) return datos[k][c] + a * (datos[k + 1][c] - datos[k][c]) end
    for i = 1, 6 do sim.setJointPosition(hJ[i], v(1 + i)) end
    local phi = v(10)
    sim.setObjectPose(hAGV, {v(8), v(9), 0, 0, 0, math.sin(phi / 2), math.cos(phi / 2)}, sim.handle_world)
    local f = datos[k]
    sim.setObjectPose(hPieza, {v(11), v(12), v(13), f[14], f[15], f[16], f[17]}, sim.handle_world)
    if t >= datos[n][1] then sim.pauseSimulation() end
end
