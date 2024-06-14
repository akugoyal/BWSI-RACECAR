class cone:
    # Data, params, dist, color priority
    DATA = []
    params = None;
    

    def inverse(self, x, a, b):
        return a / x + b

    def __init__(self):
        from scipy.optimize import curve_fit

        dists = list(zip(*self.DATA))[0]
        areas = list(zip(*self.DATA))[1]
        self.yellow_params, covar = curve_fit(self.inverse, areas, dists)

    def get_distance(self, area:int):
        return self.inverse(area, self.params[0], self.params[1])