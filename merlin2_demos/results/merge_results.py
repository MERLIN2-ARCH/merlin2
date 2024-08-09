
from os import listdir


def find_csv_filenames(path_to_dir, suffix=".csv"):
    filenames = listdir(path_to_dir)
    return [filename for filename in filenames if filename.endswith(suffix)]


f = open("result.csv", "a")
f.write("ID, World, Points, Time (Seconds), Distance (Meters)\n")

counter_id = 0

for file_path in find_csv_filenames("."):
    file = open(file_path)

    for line in file:

        if "ID" not in line:

            line_parts = line.split(",")

            new_line = str(counter_id) + "," + line_parts[1] + \
                "," + file_path.split("_")[-1].split(".")[0]
            for line_p in line_parts[2:]:
                new_line += "," + line_p

            f.write(new_line)
            counter_id += 1

f.close()
