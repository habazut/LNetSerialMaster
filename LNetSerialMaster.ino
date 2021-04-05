//////////////////////////////////////////////////////////////////////////////////
//
// This file is the main sketch for LNetSerial
//
////////////////////////////////////////////////////////////////////////////////////

/*
 *  (c) Harald Barth 2021
 *
 *  GPLv3, for the full license text, see the file LICENSE
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "LNet.h"

void setup()
{

  // Responsibility 1: Start the usb connection for diagnostics
  // This is normally Serial but uses SerialUSB on a SAMD processor
  Serial.begin(115200);

  // Start LocoNet
  LNet::begin();
}

void loop()
{
  LNet::loop();
}
