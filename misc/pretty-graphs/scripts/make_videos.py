#!/usr/bin/env python
from duckietown_utils import col_logging  # @UnusedImport
import logging
from duckietown_utils.bag_logs import d8n_get_all_images_topic
import os
logging.basicConfig()
logger = logging.getLogger(__name__)

from quickapp import QuickApp  # @UnresolvedImport

class AppExample(QuickApp):
    """ Simplest app example """

    def define_options(self, params):
        params.add_string('dir')

    def define_jobs_context(self, context):
        options = self.get_options()
        dirname = options.dir
        from conf_tools.utils import locate_files  # @UnresolvedImport
        bags = list(locate_files(dirname, pattern="*.bag", followlinks=True))
        self.info('I found %d bags in %s' % (len(bags), dirname))

        def short(f):
            return os.path.splitext(os.path.basename(f))[0]

        for f in bags:
            s = short(f)
            context.comp_dynamic(process, bag_filename=f, job_id=s)

def process(context, bag_filename):
    res = d8n_get_all_images_topic(bag_filename)
    print res


if __name__ == '__main__':
    app_example_main = AppExample.get_sys_main()
    app_example_main()
