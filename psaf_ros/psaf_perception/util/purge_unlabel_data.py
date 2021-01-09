
import csv
import os
import random


def run():
    source_dir = "/home/psaf1/project-files/training_data/carlaSpeedSigns"

    image_list = []

    percent_of_unlabled_to_keep = .10
    percent_of_validation_data = .10

    for file in os.listdir(source_dir+"/labels"):
        if file.endswith(".txt"):
            image_list.append(file.replace(".txt",""))
    print(f"Clear from unlabled files {source_dir}")
    print("Loading Data...")
    unlabled_files = []
    for fpath in image_list:
        label_file = os.path.join(source_dir,"labels/"+fpath+".txt")
        label = None
        with open(label_file, newline='') as csvfile:
            csv_in = csv.reader(csvfile, delimiter=' ', quotechar='|')
            for row in csv_in:
                if row[0] == '0':
                    label = 0
                elif row[0] == '1':
                    label = 1
                elif row[0] == '2':
                    label = 2
        if label is None:
            unlabled_files.append(fpath)
    unlabled_count_to_keep = percent_of_unlabled_to_keep * len(image_list)

    print(f"Pruge {len(unlabled_files)-unlabled_count_to_keep}...")

    while len(unlabled_files) > unlabled_count_to_keep:
        fpath = random.choice(unlabled_files)
        try:
            os.remove(os.path.join(source_dir,"labels/"+fpath+".txt"))
            os.remove(os.path.join(source_dir,"images/"+fpath+".jpg"))
        except FileNotFoundError as e:
            print(f"Unable to delete {fpath}")



    image_list.clear()
    for file in os.listdir(source_dir+"/labels"):
        if file.endswith(".txt"):
            image_list.append(file.replace(".txt",""))

    num_validation = int(len(image_list)*percent_of_validation_data)
    validation_list = random.sample(image_list,num_validation)
    validation_dir = os.path.join(source_dir,"val")
    if not os.path.exists(validation_dir):
        os.mkdir(validation_dir)
        os.mkdir(os.path.join(validation_dir,"labels"))
        os.mkdir(os.path.join(validation_dir,"images"))
    for f in validation_list:
        os.rename(os.path.join(source_dir,"labels/"+f+".txt"),os.path.join(validation_dir,"labels/"+f+".txt"))
        os.rename(os.path.join(source_dir,"images/"+f+".jpg"),os.path.join(validation_dir,"images/"+f+".jpg"))

if __name__ == "__main__":
    run()