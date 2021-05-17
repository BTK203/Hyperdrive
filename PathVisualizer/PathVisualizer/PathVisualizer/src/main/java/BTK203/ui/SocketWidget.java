package BTK203.ui;

import javax.swing.JLabel;
import javax.swing.JPanel;

import BTK203.Constants;

import java.awt.BorderLayout;

/**
 * Represents a Socket.
 */
public class SocketWidget extends JPanel {
    private static final long serialVersionUID = 1L;
    private static final String
        SOCKET_NOT_CONNECTING = "Status: Not Connected!",
        SOCKET_CONNECTING     = "Status: Searching...",
        SOCKET_NOT_RESPOND    = "Status: Not Responding!",
        SOCKET_INITALIZED     = "Status: Connected";

    private JLabel statusLabel;

    public SocketWidget() {
        super();
        setLayout(new BorderLayout());
        setBackground(Constants.ERROR_RED_COLOR);

        statusLabel = new JLabel(SOCKET_NOT_CONNECTING);
        add(statusLabel, BorderLayout.CENTER);
    }

    /**
     * Updates the widget.
     * @param bound True if the socket is bound, false otherwise.
     * @param connected True if the socket is connected, false otherwise.
     */
    public void update(boolean socketConnecting, boolean socketInitalized, boolean socketResponding) {
        if(!(socketConnecting || socketInitalized)) { //unbound, unconnected
            setBackground(Constants.ERROR_RED_COLOR);
            statusLabel.setText(SOCKET_NOT_CONNECTING);
        } else if(socketConnecting && !socketInitalized) {
            setBackground(Constants.WARNING_YELLOW_COLOR);
            statusLabel.setText(SOCKET_CONNECTING);
        } else if(socketInitalized) {
            if(socketResponding) {
                setBackground(Constants.GOOD_GREEN_COLOR);
                statusLabel.setText(SOCKET_INITALIZED);
            } else {
                setBackground(Constants.WARNING_ORANGE_COLOR);
                statusLabel.setText(SOCKET_NOT_RESPOND);
            }
        }
    }
}
