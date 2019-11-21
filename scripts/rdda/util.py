#!/usr/bin/env python

def convert_str_to_float_list(list_text):
    valid_idx_0 = list_text.find('[') + 1
    valid_idx_1 = list_text.find(']')
    result_list = map(float, list_text[valid_idx_0:valid_idx_1].split(", "))

    return result_list
