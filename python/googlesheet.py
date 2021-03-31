#from __future__ import print_function
import pickle
import os.path
import subprocess

from googleapiclient.discovery import build
from google_auth_oauthlib.flow import InstalledAppFlow
from google.auth.transport.requests import Request

out_list = []


def authen_google():

    cred_file = open('cred_file.txt', 'w')

    cred_file.write('{"installed":\
    {"client_id":"150558570819-8h6eqdrkekepjluct8e2lr0dlla1i8iu.apps.googleusercontent.com"\
    ,"project_id":"ftc18381-1614532024652","auth_uri":"https://accounts.google.com/o/oauth2/auth",\
    "token_uri":"https://oauth2.googleapis.com/token",\
    "auth_provider_x509_cert_url":"https://www.googleapis.com/oauth2/v1/certs",\
    "client_secret":"niP7SBqBxax_DwWnhPs91-N2","redirect_uris":["urn:ietf:wg:oauth:2.0:oob",\
    "http://localhost"]}}\n')

    cred_file.close()

    creds = None

    SCOPES = ['https://www.googleapis.com/auth/spreadsheets.readonly']

    if os.path.exists('token.pickle'):
        with open('token.pickle', 'rb') as token:
            creds = pickle.load(token)

    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
        else:
            flow = InstalledAppFlow.from_client_secrets_file(
                'cred_file.txt', SCOPES)
            creds = flow.run_local_server(port=0)

        with open('token.pickle', 'wb') as token:
            pickle.dump(creds, token)


    service = build('sheets', 'v4', credentials=creds)
    global input_sheet
    input_sheet = service.spreadsheets()


def process_data():

    global input_sheet
    global out_list

    # The ID and range of a sample spreadsheet.
    INPUT_SPREADSHEET_ID = '1HJDlm_dM0aTrbaaGrXf0y60Rk75DtXBudjaWaPqHri0'
    INPUT_RANGE_NAME = 'raw!A:C'

    result = input_sheet.values().get(spreadsheetId=INPUT_SPREADSHEET_ID,
                                range=INPUT_RANGE_NAME).execute()
    values = result.get('values', [])

    if not values:
        print ('No data found.')
    else:
        print ('Start to read raw data..................')
        for row in values:
            # Print columns A B and C, which correspond to indices 0 to 2.
            print('%s, %s, %s' % (row[0], row[1], row[2]))
            # process the data, do the math here
            if (row[1] == '1'):
                    outstr = '%s + %s = %d' % (row[0], row[2], int(row[0]) + int(row[2]))
                    print(outstr)
                    out_list.append(outstr)
            elif (row[1] == '2'):
                    outstr = '%s * %s = %d' % (row[0], row[2], int(row[0]) * int(row[2]))
                    print(outstr)
                    out_list.append(outstr)
            # TODO: more data need to be processed


        print ('Done with process raw data.')




def write_data():

    global out_list


    # TODO: change file name to your name
    out_file_name = "ChihChunChang"+'.txt'

    # TODO: open a new file with w (write)
    out_file = open(out_file_name, 'w')


    print ('Start to write out_list data.....................')
    for each_line in out_list:
        print ("writing:", each_line)
        # TODO: write each line to the out_file by calling write(),
        # write('\n') as new line
        out_file.write(each_line+'\n')

    print ('Done with write out_list data to file ', out_file_name)

    out_file.close()

    # open the file
    subprocess.run(['open', out_file_name], check=True)



if __name__ == '__main__':

    # IN TAKE
    authen_google()

    # PROCESS DATA
    process_data()

    # OUT TAKE
    write_data()
