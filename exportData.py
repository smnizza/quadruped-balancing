from library import *

class ExportData:
    def __init__(self) -> None:
        pass

    def export_to_csv(self, data, csv_file_path):
        if not isinstance(data[0], (list, tuple)):
            data = [data]
    
        with open(csv_file_path, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerows(data)
