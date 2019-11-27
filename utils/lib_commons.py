
import yaml
import simplejson
import glob

def read_yaml_file(file_path):
    with open(file_path, 'r') as stream:
        data = yaml.safe_load(stream)
    return data


def read_json_file(file_path):
    with open(file_path, 'r') as f:
        data = simplejson.load(f)
    return data

def get_filenames(folder, is_base_name=False):
    ''' Get all filenames under the specific folder. 
    e.g.:
        full name: data/rgb/000001.png
        base name: 000001.png 
    '''
    full_names = sorted(glob.glob(folder + "/*"))
    if is_base_name:
        base_names = [name.split("/")[-1] for name in full_names]
        return base_names
    else:
        return full_names