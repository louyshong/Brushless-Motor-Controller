#define PRINT_MESSAGE 0
#define BITCOIN_NONCE 1
#define UPDATED_KEY 2
#define DISTANCE_STATUS 3
#define SPEED_STATUS 4
#define HASH_RATE 5

typedef struct {
  uint8_t   type;
  uint8_t    number;
  float    dFloat;
  float    dFloat1;
  float    dFloat2;
  std::string message;
} mail_t;

void putMessage(int type, uint8_t number);
void putMessage(int type, double number);
void putMessage(int type, std::string message);
void putMessage(int type, double posError, double velError);
