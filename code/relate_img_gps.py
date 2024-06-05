import pandas as pd
import glob 
import sys 

def get_gps(img_name, csv_path):
    """Function finding the GPS coordinate of a given image in the image_list csv.

    Args:
        img_name (str): name of the image
        csv_path (str): path to the csv file

    Returns:
        list: list holding latitude, longitude and altitude of the image
    """
    img_list = pd.read_csv(csv_path)
    try:
        gps = img_list[img_list["id"] == img_name][["latitude", "longitude", "altitude"]].values[0]
    except IndexError:
        print(f"Image {img_name} not found in the csv file.")
        gps = None
    
    return gps

def write_txt_file(img_dir, csv_path, output_name):
    """Function writing a text file relating an image to the coordinates it was
    taken at (needed for model aligner in Colmap).

    Args:
        img_dir (str): path to the directory of the images
        csv_path (str): path to the image_list.csv file
        output_name (str): name of the output file

    Returns:
        str: name of the outputted file
    """
    all_paths = glob.glob(img_dir + "**/*.jpeg")
    if len(all_paths) == 0:
        print(f"No images found in {img_dir}")
        sys.exit(1)
    for path in all_paths:
        img_name = path.split("/")[-1].split(".")[0]
        gps = get_gps(img_name, csv_path)
        if gps is not None:
            with open(output_name, "a") as file:
                file.write(f"{img_name}.jpeg {gps[0]} {gps[1]} {gps[2]}\n")
        else:
            print(f"Image {img_name} not found in the csv file.")
    
    return output_name

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python3 relate_img_gps.py [csv_path] [img_dir] [output_name]")
        sys.exit(1)
   
    csv_path = sys.argv[1]
    img_dir = sys.argv[2]
    output_name = sys.argv[3]
    write_txt_file(img_dir, csv_path, output_name)
    print(f"File written to {output_name}.")