import asyncio
import bleak
import customtkinter

customtkinter.set_appearance_mode("dark")
customtkinter.set_default_color_theme("green")

LED_UUID = "0000ff01-0000-1000-8000-00805f9b34fb"


class App(customtkinter.CTk):
    def __init__(self, loop, interval=1/120):
        super().__init__()
        self.loop: asyncio.AbstractEventLoop = loop
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.connected = False
        self.connecting = False
        self.interval = interval
        self.tasks = []
        self.tasks.append(loop.create_task(self.ble_updater(interval)))
        self.tasks.append(loop.create_task(self.updater(interval)))

        self.geometry("150x300")
        self.title("CustomTkinter simple_example.py")

        self.frame_1 = customtkinter.CTkFrame(master=self)
        self.frame_1.pack(pady=0, padx=0, fill="both", expand=True)

        self.connect_btn = customtkinter.CTkButton(
            master=self.frame_1, text="Connect", command=self.connect_btn)
        self.connect_btn.pack(pady=10, padx=10)

        self.connect_status = customtkinter.CTkLabel(
            master=self.frame_1, text="disconnected")
        self.connect_status.pack(padx=10, pady=10)

        self.led_a = customtkinter.CTkCheckBox(
            master=self.frame_1, text="Red LED")
        self.led_a.pack(pady=10, padx=10)

        self.led_b = customtkinter.CTkCheckBox(
            master=self.frame_1, text="Green LED")
        self.led_b.pack(pady=10, padx=10)

        self.button_1 = customtkinter.CTkButton(
            master=self.frame_1, command=self.update_btn, text="Update")
        self.button_1.pack(pady=10, padx=10)

    def update_btn(self):
        mstr = f"{self.led_a.get()}{self.led_b.get()}"
        print("Button click:", mstr)
        if (self.connected):
            asyncio.ensure_future(self.client.write_gatt_char(
                LED_UUID, bytearray(mstr, 'utf-8')), loop=self.loop)

    async def connect_task(self):
        self.connected = False
        self.connecting = True
        scanned_devices = await bleak.BleakScanner.discover(1)
        print("scanned", True)

        if len(scanned_devices) == 0:
            raise bleak.exc.BleakError("no devices found")

        for device in scanned_devices:
            print(f"{device.name} ({device.address})")

        for device in scanned_devices:
            if (device.name == "ESP32 RocketTracker"):
                print("Found RocketTracker")
                try:
                    self.client = bleak.BleakClient(device)
                    await self.client.connect()
                    self.connected = True
                    for service in self.client.services:
                        print(f"  service {service.uuid}")
                        for characteristic in service.characteristics:
                            print(
                                f"  characteristic {characteristic.uuid} {hex(characteristic.handle)} ({len(characteristic.descriptors)} descriptors)"
                            )
                    self.connected
                except bleak.exc.BleakError as e:
                    print(f"  error {e}")

        self.connecting = False
        self.connect_status.configure(
            text="connected!" if self.connected else "disconnected")

    def disconnect(self):
        future = asyncio.ensure_future(
            self.client.disconnect(), loop=self.loop)

        print("disconnected")

        self.connected = False
        self.connect_status.configure(
            text="connected!" if self.connected else "disconnected")

    def connect_btn(self):
        print("connect!")
        if (not self.connecting):
            if (not self.connected):
                loop.create_task(self.connect_task())
            else:
                self.disconnect()

    async def updater(self, interval):
        while True:
            self.update()
            await asyncio.sleep(interval)

    async def ble_updater(self, interval):
        while True:
            await asyncio.sleep(interval)

    def close(self):
        # TODO: Ensure disconnect finishes
        if self.connected:
            self.disconnect()
        for task in self.tasks:
            task.cancel()
        self.loop.stop()
        self.destroy()


loop = asyncio.get_event_loop()
app = App(loop)
loop.run_forever()
loop.close()
