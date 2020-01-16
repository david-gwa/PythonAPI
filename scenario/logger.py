import logging 
import os

class logger(object):
    def __init__(self, logger_name):
        self.log = logging.getLogger(logger_name)
        self.log.setLevel(logging.DEBUG)
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.DEBUG)
        self.log.addHandler(console_handler)
    
    # only call in ScenarioRunner
    def set_output_file(self, filename=None, dirname=None):
        if dirname :
            self.dirname = dirname 
        else:
            self.dirname = "outputs"

        if filename :
            self.filename = filename 
        else:
            self.filename = "Record.log"
        try:
            os.makedirs(self.dirname)
        except:
            pass
        self.file_path =  os.path.join(self.dirname, self.filename)
        file_handler = logging.FileHandler(self.file_path)
        file_handler.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)
        self.log.addHandler(file_handler)

#TODO: scenario_manager should has all data info about the running scenario, reconstruct the scenario_manager module


