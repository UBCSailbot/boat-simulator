class Dataline:
    def __init__(self, data, noise_generator) -> None:
        self.__data = data
        self.__noise_generator = noise_generator

    @property
    def noise_generator(self):
        return self.__noise_generator

    def read(self):
        return self.__data

    def update(self, new_data):
        self.__data = new_data

    def noisemaker(self):
        return self.__data + self.__noise_generator.next()
