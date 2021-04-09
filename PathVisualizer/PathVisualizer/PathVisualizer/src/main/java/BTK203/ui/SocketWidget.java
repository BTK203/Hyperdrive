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
     * @param socketConnecting {@code true} if the socket is actively connecting, {@code false} otherwise.
     * @param connected {@code true} if the socket is connected, {@code false} otherwise.
     */
    public void update(boolean socketConnecting, boolean socketInitalized) {
        if(!(socketConnecting || socketInitalized)) { //unbound, unconnected
            setBackground(Constants.ERROR_RED_COLOR);
            statusLabel.setText(SOCKET_NOT_CONNECTING);
        } else if(socketConnecting && !socketInitalized) {
            setBackground(Constants.WARNING_YELLOW_COLOR);
            statusLabel.setText(SOCKET_CONNECTING);
        } else if(socketInitalized) {
            setBackground(Constants.GOOD_GREEN_COLOR);
            statusLabel.setText(SOCKET_INITALIZED);
        }
    }
}
