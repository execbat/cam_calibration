#import socket
import io
import xml.etree.ElementTree as xml
#from IPython import display

#import time



def extract_xml(bytes_parcel, find = 'all'): # 'find' could be also the list of key-words
    res_dict = dict()
       
    try: # checkking if we can parse input data to ElementTree
        tree = xml.parse(io.BytesIO(bytes_parcel)) # xml.etree.ElementTree.ElementTree

    except TypeError:
        print('incorrect type of input data')

    else:
        try: # checking 'find'
            find = find.lower()
            

        except AttributeError: #operating with list
            root = tree.getroot()
            
            #print('cant make lowercase')
            for i_tag in find:
                elem = tree.find(i_tag)                
                try: # if incorrect key_word in list
                    if elem.text is not None:
                        res_dict[elem.tag] = elem.text    # if element contains text
                    else:
                        res_dict[elem.tag] = elem.attrib  # if element contains dict
                except AttributeError:
                    print('Data do not contain "{}" key-word'.format(i_tag))

        else: #checking ALL
            if find == 'all':
                root = tree.getroot()
                if root.text is not None:        
                    res_dict[root.tag] = root.text    # if element contains text
                else:
                    res_dict[root.tag] = root.attrib  # if element contains dict

                for elem in root:
                    if elem.text is not None:        
                        res_dict[elem.tag] = elem.text    # if element contains text
                    else:
                        res_dict[elem.tag] = elem.attrib  # if element contains dict
            else:
                raise AttributeError('Use "all" or the list of keywords')            

    return res_dict


# Creates structure for ambedding into the xml package, which will be send to robot
def create_xml(sent_mess_list):
    
    for num, elem in enumerate(sent_mess_list):
        key = list(elem.keys())[0]
        value = elem[key]
        if type(value) == dict: 
            if num == 0: 
                root = xml.Element(key , elem[key])
            else:
                element_tree = xml.SubElement(root, key , elem[key])
                
        elif type(value) == str:
            if num == 0:
                root = xml.Element(key)
                root.text = value 
            else:
                element_tree = xml.SubElement(root, key)
                element_tree.text = value                 
            
    tree = xml.ElementTree(root)   
    DecodedTree = xml.tostring(root, encoding='utf8', method='xml', short_empty_elements='true', xml_declaration=False)
    
    return(DecodedTree) 
    
    
def create_xml_fast(sent_mess_list):
    xml_parts = []

    # Обрабатываем первый элемент отдельно — это корневой тег
    first_elem = sent_mess_list[0]
    root_key = list(first_elem.keys())[0]
    root_value = first_elem[root_key]

    if isinstance(root_value, dict):
        attrs = ' '.join(f'{k}="{v}"' for k, v in root_value.items())
        xml_parts.append(f'<{root_key} {attrs}>')
    else:
        xml_parts.append(f'<{root_key}>{root_value}')

    # Остальные элементы — вложенные
    for elem in sent_mess_list[1:]:
        key = list(elem.keys())[0]
        value = elem[key]

        if isinstance(value, dict):
            attrs = ' '.join(f'{k}="{v}"' for k, v in value.items())
            xml_parts.append(f'<{key} {attrs} />')
        else:
            xml_parts.append(f'<{key}>{value}</{key}>')

    # Закрывающий тег для корня
    xml_parts.append(f'</{root_key}>')

    return ''.join(xml_parts).encode('utf-8')
