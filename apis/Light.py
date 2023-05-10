class Light:

    def __init__(self):
        import Led
        self.led = Led.Led()
        self.clear()

    def set_yellow(self):
        self.set_color(255, 255, 0)

    def set_green(self):
        self.set_color(0, 255, 0)

    def set_red(self):
        self.set_color(255, 0, 0)

    def set_blue(self):
        self.set_color(0, 0, 255)

    def clear(self):
        self.set_color(0, 0, 0)

    def set_color(self, r, g, b):
        self.led.ledIndex(0x1 ^ 0x2 ^ 0x4 ^ 0x8, r, g, b)
