#include "../include/sync.h"

void dela_(clock_t n) {
	clock_t start = clock();
	while (clock() - start < n);
}


syncPi::syncPi() {
	
}
syncPi::~syncPi() {
	ssh_disconnect(this->sync_pi);

	ssh_free(this->sync_pi);


}

int syncPi::verify_knownhost()
{
	enum ssh_known_hosts_e state;
	unsigned char *hash = NULL;
	ssh_key srv_pubkey = NULL;
	size_t hlen;
	char buf[10];
	char *hexa;
	char *p;
	int cmp;
	int rc;

	rc = ssh_get_server_publickey(this->sync_pi, &srv_pubkey);
	if (rc < 0) {
		return -1;
	}

	rc = ssh_get_publickey_hash(srv_pubkey,
		SSH_PUBLICKEY_HASH_SHA1,
		&hash,
		&hlen);
	ssh_key_free(srv_pubkey);
	if (rc < 0) {
		return -1;
	}

	state = ssh_session_is_known_server(this->sync_pi);
	switch (state) {
	case SSH_KNOWN_HOSTS_OK:
		/* OK */

		break;
	case SSH_KNOWN_HOSTS_CHANGED:
		fprintf(stderr, "Host key for server changed: it is now:\n");
		ssh_print_hexa("Public key hash", hash, hlen);
		fprintf(stderr, "For security reasons, connection will be stopped\n");
		ssh_clean_pubkey_hash(&hash);

		return -1;
	case SSH_KNOWN_HOSTS_OTHER:
		fprintf(stderr, "The host key for this server was not found but an other"
			"type of key exists.\n");
		fprintf(stderr, "An attacker might change the default server key to"
			"confuse your client into thinking the key does not exist\n");
		ssh_clean_pubkey_hash(&hash);

		return -1;
	case SSH_KNOWN_HOSTS_NOT_FOUND:
		fprintf(stderr, "Could not find known host file.\n");
		fprintf(stderr, "If you accept the host key here, the file will be"
			"automatically created.\n");

		/* FALL THROUGH to SSH_SERVER_NOT_KNOWN behavior */

	case SSH_KNOWN_HOSTS_UNKNOWN:
		hexa = ssh_get_hexa(hash, hlen);
		fprintf(stderr, "The server is unknown. Do you trust the host key?\n");
		fprintf(stderr, "Public key hash: %s\n", hexa);
		ssh_string_free_char(hexa);
		ssh_clean_pubkey_hash(&hash);
		p = fgets(buf, sizeof(buf), stdin);
		if (p == NULL) {
			return -1;
		}

		cmp = strncasecmp(buf, "yes", 3);
		if (cmp != 0) {
			return -1;
		}

		rc = ssh_session_update_known_hosts(this->sync_pi);
		if (rc < 0) {
			fprintf(stderr, "Error %s\n", strerror(errno));
			return -1;
		}

		break;
	case SSH_KNOWN_HOSTS_ERROR:
		fprintf(stderr, "Error %s", ssh_get_error(this->sync_pi));
		ssh_clean_pubkey_hash(&hash);
		return -1;
	}

	ssh_clean_pubkey_hash(&hash);
	return 0;
}

int syncPi::pinModeOutput(){
	ssh_channel channel;
		int rc;
		channel = ssh_channel_new(this->sync_pi);
		if (channel == NULL) return SSH_ERROR;
		rc = ssh_channel_open_session(channel);

		if (rc != SSH_OK) {
			ssh_channel_free(channel);
			return rc;
		}

		rc = ssh_channel_request_exec(channel, "raspi-gpio set 2 op");

		if (rc != SSH_OK) {
			ssh_channel_close(channel);
			ssh_channel_free(channel);
			return rc;
		}
		ssh_channel_send_eof(channel);
		ssh_channel_close(channel);
		ssh_channel_free(channel);

		return rc;
}

int syncPi::remote_gpio(int gpio_num, bool is_high) {
	ssh_channel channel;
	int rc;
	channel = ssh_channel_new(this->sync_pi);
	if (channel == NULL) return SSH_ERROR;
	rc = ssh_channel_open_session(channel);

	if (rc != SSH_OK) {
		ssh_channel_free(channel);
		return rc;
	}
	if(gpio_num == 2){
		if (is_high) {
			rc = ssh_channel_request_exec(channel, "raspi-gpio set 2 dh" );
		}
		else {
			rc = ssh_channel_request_exec(channel, "raspi-gpio set 2 dl");
		}
	}
	else if (gpio_num == 3){
		if (is_high) {
			rc = ssh_channel_request_exec(channel, "raspi-gpio set 3 dh" );
		}
		else {
			rc = ssh_channel_request_exec(channel, "raspi-gpio set 3 dl");
		}
	}
	if (rc != SSH_OK) {
		ssh_channel_close(channel);
		ssh_channel_free(channel);
		return rc;
	}
	ssh_channel_send_eof(channel);
	ssh_channel_close(channel);
	ssh_channel_free(channel);

	return rc;
}

int syncPi::establish_connect(const char *ip) {

	if (this->sync_pi == NULL) exit(-1);

	ssh_options_set(this->sync_pi, SSH_OPTIONS_HOST, ip);

	rc = ssh_connect(this->sync_pi);
	if (rc != SSH_OK) {
		fprintf(stderr, "ERROR connecting to localhost : %s\n", ssh_get_error(this->sync_pi));
		ssh_free(this->sync_pi);
		exit(-1);
	}
	if (verify_knownhost() < 0) {
		ssh_disconnect(this->sync_pi);
		ssh_free(this->sync_pi);
		exit(-1);
	}

	password = getpass("Password: ");
	rc = ssh_userauth_password(this->sync_pi, NULL, password);
	if (rc != SSH_AUTH_SUCCESS) {
		fprintf(stderr, "Error authenticationg with password : %s\n", ssh_get_error(this->sync_pi));
		ssh_disconnect(this->sync_pi);
		ssh_free(this->sync_pi);
		exit(-1);
	}
	well_established = true;


	return 1;
}

int syncPi::send_square(int pin, bool is_high){
	if (well_established == false) {
		return -1;
	}
	this->remote_gpio(pin, is_high);
	return 1;
}

int syncPi::get_endsign() {
	this->continue_signal = false;
	return 1;
}
