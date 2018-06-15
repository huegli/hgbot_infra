#!/usr/bin/env python

import os
import requests
import sys
from subprocess import call


def blink_activity():
    try:
        supervisor_url = os.environ["RESIN_SUPERVISOR_ADDRESS"]
        api_key = os.environ["RESIN_SUPERVISOR_API_KEY"]
    except KeyError:
        print "Can't find RESIN environment variables, run from resin.io shell"
        sys.exit(1)

    blink_url = ('"' + supervisor_url + '/v1/blink'
                 '?apikey=' + api_key + '"')
    curl_cmd = ('curl -X POST --header '
                '"Content-Type:application/json" ' + blink_url)

    print blink_url
    print curl_cmd

    call(curl_cmd, shell=True)


def blink_rpi_activity():
    try:
        supervisor_url = os.environ["RESIN_SUPERVISOR_ADDRESS"]
        api_key = os.environ["RESIN_SUPERVISOR_API_KEY"]
    except KeyError:
        print "Can't find RESIN environment variables, run from resin.io shell"
        sys.exit(1)

    headers = {
            'Content-Type': 'application/json',
    }

    params = (
            ('apikey', api_key),
    )

    blink_url = supervisor_url + '/v1/blink'

    response = requests.post(blink_url, headers=headers, params=params)
