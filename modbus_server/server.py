import logging
import math
import random
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from enum import IntEnum

from pymodbus.datastore import ModbusDeviceContext, ModbusSequentialDataBlock, ModbusServerContext
from pymodbus.server import StartTcpServer


class Register(IntEnum):
    TEMPERATURE = 0
    PRESSURE = 1
    MACHINE_STATE = 2
    CYCLE_COUNT = 3
    ALARM_CODE = 4
    UPTIME_HI = 5
    UPTIME_LO = 6
    HUMIDITY = 7
    VIBRATION = 8
    POWER_KW = 9


class MachineState(IntEnum):
    OFF = 0
    IDLE = 1
    WARMING = 2
    RUNNING = 3
    COOLING = 4
    ERROR = 5


class AlarmCode(IntEnum):
    OK = 0
    OVERHEAT = 1
    OVERPRESSURE = 2
    VIBRATION_HIGH = 3


FSM_TRANSITIONS = {
    MachineState.OFF: [MachineState.IDLE],
    MachineState.IDLE: [MachineState.OFF, MachineState.WARMING],
    MachineState.WARMING: [MachineState.RUNNING, MachineState.ERROR],
    MachineState.RUNNING: [MachineState.COOLING, MachineState.ERROR],
    MachineState.COOLING: [MachineState.IDLE],
    MachineState.ERROR: [MachineState.IDLE],
}

FSM_MIN_DURATION = {
    MachineState.OFF: 6,
    MachineState.IDLE: 8,
    MachineState.WARMING: 10,
    MachineState.RUNNING: 12,
    MachineState.COOLING: 8,
    MachineState.ERROR: 6,
}

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s - %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("modbus.server")


@dataclass
class MachineSimulator:
    state: MachineState = MachineState.IDLE
    state_entered_at: float = field(default_factory=time.time)
    start_time: float = field(default_factory=time.time)
    temperature: float = 22.0
    pressure: float = 5.0
    humidity: float = 55.0
    vibration: float = 0.2
    power_kw: float = 0.8
    cycle_count: int = 0
    alarm_code: int = 0
    pressure_phase: float = 0.0

    def update(self) -> None:
        now = time.time()
        elapsed = now - self.state_entered_at

        if elapsed >= FSM_MIN_DURATION[self.state]:
            candidates = FSM_TRANSITIONS[self.state]
            if self.state == MachineState.RUNNING and random.random() < 0.08:
                next_state = MachineState.ERROR
                self.alarm_code = int(self._pick_alarm_for_error())
            else:
                next_state = random.choice(candidates)
                if next_state == MachineState.ERROR:
                    self.alarm_code = int(self._pick_alarm_for_error())
                else:
                    self.alarm_code = int(AlarmCode.OK)

            if next_state != self.state:
                if self.state == MachineState.RUNNING and next_state == MachineState.COOLING:
                    self.cycle_count += 1
                if next_state == MachineState.ERROR:
                    log.info("FSM: %s -> %s [alarm=%s]", self.state.name, next_state.name, AlarmCode(self.alarm_code).name)
                else:
                    log.info("FSM: %s -> %s", self.state.name, next_state.name)
                self.state = next_state
                self.state_entered_at = now

        target_temperature = {
            MachineState.OFF: 20.0,
            MachineState.IDLE: 23.0,
            MachineState.WARMING: 65.0,
            MachineState.RUNNING: 82.0,
            MachineState.COOLING: 35.0,
            MachineState.ERROR: 95.0,
        }[self.state]
        self.temperature += (target_temperature - self.temperature) * 0.08
        self.temperature = round(max(18.0, min(110.0, self.temperature + random.gauss(0, 0.3))), 2)

        base_pressure = {
            MachineState.OFF: 0.0,
            MachineState.IDLE: 5.0,
            MachineState.WARMING: 8.0,
            MachineState.RUNNING: 12.0,
            MachineState.COOLING: 7.0,
            MachineState.ERROR: 14.5,
        }[self.state]
        self.pressure_phase += 0.3
        self.pressure = round(max(0.0, base_pressure + math.sin(self.pressure_phase) * 0.4 + random.gauss(0, 0.1)), 2)

        humidity_target = {
            MachineState.OFF: 50.0,
            MachineState.IDLE: 55.0,
            MachineState.WARMING: 48.0,
            MachineState.RUNNING: 44.0,
            MachineState.COOLING: 60.0,
            MachineState.ERROR: 70.0,
        }[self.state]
        self.humidity += (humidity_target - self.humidity) * 0.05
        self.humidity = round(max(30.0, min(90.0, self.humidity + random.gauss(0, 0.2))), 2)

        self.vibration = round(max(0.0, {
            MachineState.OFF: 0.0,
            MachineState.IDLE: 0.2,
            MachineState.WARMING: 1.5,
            MachineState.RUNNING: 4.5,
            MachineState.COOLING: 1.0,
            MachineState.ERROR: 8.0,
        }[self.state] + random.gauss(0, 0.15)), 2)

        self.power_kw = round(max(0.0, {
            MachineState.OFF: 0.0,
            MachineState.IDLE: 0.8,
            MachineState.WARMING: 5.0,
            MachineState.RUNNING: 12.5,
            MachineState.COOLING: 2.0,
            MachineState.ERROR: 1.5,
        }[self.state] + random.gauss(0, 0.2)), 2)

        self._sync_alarm_with_process_values()

    def _pick_alarm_for_error(self) -> AlarmCode:
        weights = [
            (AlarmCode.OVERHEAT, max(self.temperature, 30.0)),
            (AlarmCode.OVERPRESSURE, max(self.pressure * 8, 10.0)),
            (AlarmCode.VIBRATION_HIGH, max(self.vibration * 12, 8.0)),
        ]
        alarms = [alarm for alarm, _weight in weights]
        alarm_weights = [weight for _alarm, weight in weights]
        return random.choices(alarms, weights=alarm_weights, k=1)[0]

    def _sync_alarm_with_process_values(self) -> None:
        if self.state != MachineState.ERROR:
            self.alarm_code = int(AlarmCode.OK)
            return

        if self.temperature >= 90:
            self.alarm_code = int(AlarmCode.OVERHEAT)
        elif self.pressure >= 14:
            self.alarm_code = int(AlarmCode.OVERPRESSURE)
        elif self.vibration >= 7:
            self.alarm_code = int(AlarmCode.VIBRATION_HIGH)
        elif self.alarm_code == int(AlarmCode.OK):
            self.alarm_code = int(self._pick_alarm_for_error())

    def to_registers(self) -> list[int]:
        uptime = int(time.time() - self.start_time)
        return [
            int(self.temperature * 100),
            int(self.pressure * 100),
            int(self.state),
            self.cycle_count & 0xFFFF,
            self.alarm_code,
            (uptime >> 16) & 0xFFFF,
            uptime & 0xFFFF,
            int(self.humidity * 100),
            int(self.vibration * 100),
            int(self.power_kw * 10),
        ]


def simulation_loop(context: ModbusServerContext, machine: MachineSimulator) -> None:
    while True:
        machine.update()
        context[0].setValues(3, 0, machine.to_registers())
        log.info(
            "[%s] T=%.2fC P=%.2fbar Hum=%.2f%% Vib=%.2f Power=%.2fkW Cycles=%s Alarm=%s",
            machine.state.name,
            machine.temperature,
            machine.pressure,
            machine.humidity,
            machine.vibration,
            machine.power_kw,
            machine.cycle_count,
            machine.alarm_code,
        )
        time.sleep(2)


def main() -> None:
    block = ModbusSequentialDataBlock(0, [0] * 16)
    store = ModbusDeviceContext(hr=block)
    context = ModbusServerContext(devices=store, single=True)

    machine = MachineSimulator()
    thread = threading.Thread(target=simulation_loop, args=(context, machine), daemon=True)
    thread.start()

    def shutdown(_sig, _frame) -> None:
        log.info("A encerrar servidor Modbus...")
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    log.info("Servidor Modbus TCP a escutar em 0.0.0.0:5020")
    StartTcpServer(context=context, address=("0.0.0.0", 5020))


if __name__ == "__main__":
    main()
