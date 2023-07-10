from bluerov_implementation import SimpleBluerovImplementation, ArduSubBluerovImplementation

class Bluerov:
    def __init__(self, implementation="simple"):
        """
            implementation : choose if you use ArduSub or not ("ardusub" or "simple")
        """
        if implementation == "simple":
            self.implementation = SimpleBluerovImplementation()
        elif implementation == "ardusub":
            self.implementation = ArduSubBluerovImplementation()
        else : 
            raise ValueError("Incorrect implementation value. Choose 'ardusub' or 'simple'.")

    def move_to(self, coordinates, is_init=False):
        self.implementation.move_to(coordinates, is_init)

    def get_current_position(self):
        return self.implementation.get_current_position()
    
    def get_distance_made(self):
        self.implementation.get_distance_made()
