
import customtkinter
import tkintermapview
from dotenv import load_dotenv
import os

load_dotenv()


class App(customtkinter.CTk):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._set_appearance_mode("System")

        self.geometry("512x512")
        self.title("Map Display")

        self.frame_1 = customtkinter.CTkFrame(master=self)
        self.frame_1.pack(pady=0, padx=0, fill="both", expand=True)

        self.map = tkintermapview.TkinterMapView(master=self.frame_1,)
        self.map.set_tile_server(
            "https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)
        self.map.set_position(deg_x=float(os.getenv("LATITUDE", 0)),
                              deg_y=-float(os.getenv("LONGITUDE", 0)))
        self.map.pack(padx=0, pady=0, fill="both", expand=True)

    def close(self):
        self.destroy()

    def start(self):
        self.mainloop()


app = App()
app.start()
